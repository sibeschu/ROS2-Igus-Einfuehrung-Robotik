#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType

import curses
import threading
import time
from typing import Dict, Set

KEY_HOLD_TIMEOUT = 0.2  # seconds to consider a key still held after last read
STATUS_REFRESH = 0.1    # seconds between UI refreshes
MOVEMENT_TIMER_PERIOD = 0.1  # seconds (10 Hz)


class KeyboardMoveSample(Node):
    def __init__(self):
        super().__init__('keyboard_control_curses')

        # Publishers
        self.twist_topic = '/delta_twist_cmds'
        self.joint_topic = '/delta_joint_cmds'
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_pub = self.create_publisher(JointJog, self.joint_topic, 10)

        # State
        self.joint_mode = True  # True=joint mode, False=twist mode
        self.selected_joint = 0
        self.velocity = 0.01

        # active_keys: map keycode -> last_seen_time
        self.active_keys: Dict[int, float] = {}
        self.ak_lock = threading.Lock()

        # Joint names
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Timer for continuous movement (runs in ROS thread)
        self.movement_timer = self.create_timer(MOVEMENT_TIMER_PERIOD, self.movement_callback)

        self.mode_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        # Set initial command type depending on mode
        self.set_command_type(0 if not self.joint_mode else 1)

    # ---------- Public helpers for UI thread ----------
    def key_seen(self, key_code: int):
        """Called by UI when a key was read. Records the key with timestamp."""
        with self.ak_lock:
            self.active_keys[key_code] = time.monotonic()

    def keys_current(self) -> Set[int]:
        """Return set of keys currently considered held (by timeout)."""
        with self.ak_lock:
            now = time.monotonic()
            # filter by timeout
            keys = {k for k, t in self.active_keys.items() if now - t <= KEY_HOLD_TIMEOUT}
            # remove stale keys from dict to keep it small
            stale = [k for k, t in self.active_keys.items() if now - t > KEY_HOLD_TIMEOUT]
            for k in stale:
                del self.active_keys[k]
            return keys

    def clear_keys(self):
        with self.ak_lock:
            self.active_keys.clear()

    # ---------- Movement & publishing ----------
    def set_command_type(self, cmd_type: int):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Servo switch_command_type service not available")
            return
        req = ServoCommandType.Request()
        req.command_type = cmd_type
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Command type set to {cmd_type}")
        else:
            self.get_logger().error(f"Failed to set command type: {future.exception()}")

    def movement_callback(self):
        """Called periodically by rclpy timer to publish movement based on active keys."""
        keys = self.keys_current()
        if not keys:
           return

        # Joint mode
        if self.joint_mode:
            for k in keys:
                # arrow up/down: move selected joint
                if k == curses.KEY_UP:
                    self.move_joint(self.selected_joint, self.velocity)
                elif k == curses.KEY_DOWN:
                    self.move_joint(self.selected_joint, -self.velocity)
        else:
            # Twist mode: aggregate twist per timer tick
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            lx = ly = lz = 0.0
            for k in keys:
                if k == curses.KEY_UP:
                    lx += self.velocity
                elif k == curses.KEY_DOWN:
                    lx -= self.velocity
                elif k == curses.KEY_LEFT:
                    ly += self.velocity
                elif k == curses.KEY_RIGHT:
                    ly -= self.velocity
                elif k == curses.KEY_HOME:
                    lz += self.velocity
                elif k == curses.KEY_END:
                    lz -= self.velocity
            msg.twist.linear.x = lx
            msg.twist.linear.y = ly
            msg.twist.linear.z = lz
            if lx or ly or lz:
                self.twist_pub.publish(msg)

    def move_joint(self, joint_idx: int, velocity: float):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = [self.joint_names[joint_idx]]
        msg.velocities = [velocity]
        msg.duration = 0.0
        self.joint_pub.publish(msg)

    def stop_all(self):
        """Publish zero commands to stop robot (called on exit or when no keys)."""
        # stop joint
        for j in range(len(self.joint_names)):
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = [self.joint_names[j]]
            msg.velocities = [0.0]
            msg.duration = 0.0
            # publish multiple times for reliability
            for _ in range(2):
                self.joint_pub.publish(msg)
        # stop twist
        tmsg = TwistStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = 'base_link'
        for _ in range(2):
            self.twist_pub.publish(tmsg)

    def emergency_stop(self):
        self.get_logger().warn("EMERGENCY STOP triggered (keyboard).")
        self.stop_all()

    def toggle_mode(self):
        self.joint_mode = not self.joint_mode
        cmd_type = 0 if self.joint_mode else 1  # JOINT_JOG or TWIST
        self.set_command_type(cmd_type)

    # ---------- Status for UI ----------
    def get_status(self) -> Dict:
        """Return a dict of current status values for UI rendering."""
        mode = "JOINT" if self.joint_mode else "TWIST"
        joint = (f"joint{self.selected_joint + 1}" if self.joint_mode else "N/A")
        with self.ak_lock:
            active = list(self.active_keys.keys())
        return {
            "node_name": self.get_name(),
            "mode": mode,
            "selected_joint": joint,
            "velocity": self.velocity,
            "active_keys": active,
            "publishes": [self.joint_topic, self.twist_topic],
        }


# ---------- Curses UI main loop ----------
def curses_main(stdscr, node: KeyboardMoveSample):
    # Curses init
    curses.curs_set(0)
    stdscr.nodelay(True)  # non-blocking getch
    stdscr.keypad(True)
    stdscr.clear()

    # small helper to render centered header
    def header(text, line=0):
        stdscr.attron(curses.A_BOLD)
        stdscr.addstr(line, 2, text)
        stdscr.attroff(curses.A_BOLD)

    last_status_update = 0.0
    try:
        while rclpy.ok():
            now = time.time()
            # read input (non-blocking)
            try:
                k = stdscr.getch()
            except KeyboardInterrupt:
                break

            if k != -1:
                # ESC to quit
                if k == 27:
                    break

                # page up/down -> change velocity
                if k == curses.KEY_PPAGE:  # Page Up
                    node.velocity += 0.01
                elif k == curses.KEY_NPAGE:  # Page Down
                    node.velocity = max(0.001, node.velocity - 0.01)
                elif k in (ord('m'), ord('M')):
                    node.toggle_mode()
                elif k in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')):
                    node.selected_joint = int(chr(k)) - 1
                elif k == ord(' '):
                    node.emergency_stop()
                    node.clear_keys()
                else:
                    # record any other keypress for movement (arrows, home, end)
                    # curses returns named keys as constants (curses.KEY_*)
                    node.key_seen(k)

            # Also: if no key read, keep last_seen timestamps; movement_callback handles timeout
            # Render status periodically (or every loop)
            if now - last_status_update >= STATUS_REFRESH:
                status = node.get_status()

                stdscr.erase()
                header("=== Roboter Tastatur-Steuerung (curses) ===", 0)
                stdscr.addstr(2, 2, f"Node: {status['node_name']}")
                stdscr.addstr(3, 2, f"Publishes to: {', '.join(status['publishes'])}")

                stdscr.addstr(5, 2, "=== Einstellungen ===")
                stdscr.addstr(6, 4, f"Modus: {status['mode']}")
                stdscr.addstr(7, 4, f"Ausgewähltes Gelenk: {status['selected_joint']}")
                stdscr.addstr(8, 4, f"Geschwindigkeit: {status['velocity']:.3f}")

                stdscr.addstr(10, 2, "=== Steuerung ===")
                stdscr.addstr(11, 4, "Pfeiltasten: Bewegung (halten möglich via autorepeat)")
                stdscr.addstr(12, 4, "Leertaste: Notfall-Stop")
                stdscr.addstr(13, 4, "m: Modus wechseln (Joint/Twist)")
                stdscr.addstr(14, 4, "1-6: Joint wählen")
                stdscr.addstr(15, 4, "PageUp/PageDown: Geschwindigkeit anpassen")
                stdscr.addstr(16, 4, "ESC: Beenden")

                stdscr.addstr(18, 2, "=== Aktive Tasten (letzte 200 ms) ===")
                keys = status['active_keys']
                if keys:
                    # show readable names for arrow/home/end if present
                    readable = []
                    for kk in keys:
                        if kk == curses.KEY_UP:
                            readable.append("UP")
                        elif kk == curses.KEY_DOWN:
                            readable.append("DOWN")
                        elif kk == curses.KEY_LEFT:
                            readable.append("LEFT")
                        elif kk == curses.KEY_RIGHT:
                            readable.append("RIGHT")
                        elif kk == curses.KEY_HOME:
                            readable.append("HOME")
                        elif kk == curses.KEY_END:
                            readable.append("END")
                        else:
                            # printable?
                            try:
                                ch = chr(kk)
                                if ch.isprintable():
                                    readable.append(ch)
                                else:
                                    readable.append(str(kk))
                            except Exception:
                                readable.append(str(kk))
                    stdscr.addstr(19, 4, ", ".join(readable))
                else:
                    stdscr.addstr(19, 4, "—")

                stdscr.refresh()
                last_status_update = now

            # small sleep to avoid busy loop
            time.sleep(0.01)

    finally:
        # on exit make sure robot stops
        node.get_logger().info("Exiting UI, sending stops...")
        node.stop_all()


def main():
    rclpy.init()
    node = KeyboardMoveSample()

    # start rclpy.spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run curses UI in main thread
    try:
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.get_logger().info("Shutting down node...")
        try:
            node.cleanup_before_exit  # keep existence; if you implemented cleanup_before_exit, call it
        except Exception:
            pass
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()
        print("Roboter-Steuerung beendet.")


if __name__ == '__main__':
    main()
