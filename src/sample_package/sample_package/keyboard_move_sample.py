import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import SetBool
from pynput import keyboard
import sys
import termios
import tty
import threading
import os
import select
import time

class KeyboardMoveSample(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)
        
        # Mode switching service client
        self.mode_client = self.create_client(SetBool, '/servo_node/switch_command_type')
        
        # State variables
        self.joint_mode = True  # True for joint mode, False for twist mode
        self.selected_joint = 0  # 0-5 for joints 1-6
        self.velocity = 0.01
        self.active_keys = set()
        
        # Joint names
        self.joint_names = [f'joint{i+1}' for i in range(6)]
        
        # Create a timer for continuous movement
        self.movement_timer = self.create_timer(0.1, self.movement_callback)  # 10Hz update rate
        
        # Set up keyboard listener with focus on terminal
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
            suppress=True  # This will capture all keyboard events when focused
        )
        listener.start()

    def keyboard_listener(self):
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            if key not in self.active_keys:
                self.active_keys.add(key)
                self.get_logger().info(f'Key pressed: {key}')  # Add this debug line
                self.handle_key_press(key)
        except Exception as e:
            self.get_logger().error(f'Error handling key press: {e}')

    def on_release(self, key):
        try:
            self.active_keys.discard(key)
            if not self.active_keys:  # Only stop if no keys are pressed
                self.stop_movement()
        except Exception as e:
            self.get_logger().error(f'Error handling key release: {e}')

    def handle_key_press(self, key):
        if key == keyboard.Key.space:
            self.emergency_stop()
            return

        if key == keyboard.Key.page_up:
            self.velocity += 0.01
            self.print_state()
            return
        elif key == keyboard.Key.page_down:
            self.velocity = max(0.01, self.velocity - 0.01)  # Prevent negative velocity
            self.print_state()
            return

        if hasattr(key, 'char'):
            if key.char in '123456':
                self.selected_joint = int(key.char) - 1
                self.print_state()
            elif key.char == 'm':
                self.toggle_mode()
                return

    def handle_joint_movement(self, key):
        if key == keyboard.Key.up:
            self.move_joint(self.selected_joint, self.velocity)
        elif key == keyboard.Key.down:
            self.move_joint(self.selected_joint, -self.velocity)

    def handle_twist_movement(self, key):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if key == keyboard.Key.up:
            msg.twist.linear.x = self.velocity
        elif key == keyboard.Key.down:
            msg.twist.linear.x = -self.velocity
        elif key == keyboard.Key.left:
            msg.twist.linear.y = self.velocity
        elif key == keyboard.Key.right:
            msg.twist.linear.y = -self.velocity
        elif key == keyboard.Key.home:  # Changed from page_up
            msg.twist.linear.z = self.velocity
        elif key == keyboard.Key.end:   # Changed from page_down
            msg.twist.linear.z = -self.velocity

        self.twist_pub.publish(msg)

    def move_joint(self, joint_idx, velocity):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = [self.joint_names[joint_idx]]
        msg.velocities = [velocity]
        msg.duration = 0.0  # continuous motion
        self.joint_pub.publish(msg)

    def stop_movement(self):
        if self.joint_mode:
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = [self.joint_names[self.selected_joint]]
            msg.velocities = [0.0]
            msg.duration = 0.0
            # Publish stop command multiple times to ensure it's received
            for _ in range(3):
                self.joint_pub.publish(msg)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
        else:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            # Publish stop command multiple times to ensure it's received
            for _ in range(3):
                self.twist_pub.publish(msg)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))

    def emergency_stop(self):
        self.get_logger().warn("EMERGENCY STOP!")
        self.stop_movement()
        # Add a small delay to ensure stop commands are processed
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def toggle_mode(self):
        self.joint_mode = not self.joint_mode
        req = SetBool.Request()
        req.data = self.joint_mode
        self.mode_client.call_async(req)
        self.print_state()

    def print_state(self):
        mode = "JOINT" if self.joint_mode else "TWIST"
        joint = f"joint{self.selected_joint + 1}" if self.joint_mode else "N/A"
        
        # Clear screen
        os.system('clear')
        
        # Print header
        print("\033[1m" + "=== Roboter Tastatur-Steuerung aktiv ===" + "\033[0m")
        print("Drücken Sie Strg+C zum Beenden\n")
        
        # Print current status with colors
        print("\033[1m=== Aktuelle Einstellungen ===\033[0m")
        print(f"Modus: \033[92m{mode}\033[0m")
        print(f"Ausgewähltes Gelenk: \033[92m{joint}\033[0m")
        print(f"Geschwindigkeit: \033[92m{self.velocity:.3f}\033[0m")
        
        # Print controls
        print("\n\033[1m=== Steuerung ===\033[0m")
        print("Pfeiltasten: Bewegung")
        print("Leertaste: Notfall-Stop")
        print("m: Modus wechseln (Joint/Twist)")
        print("1-6: Joint auswählen")
        print("Bild auf/ab: Geschwindigkeit anpassen")
        
        # Print active keys if any
        if self.active_keys:
            print("\n\033[1m=== Aktive Tasten ===\033[0m")
            for key in self.active_keys:
                print(f"- {key}")
            
        self.get_logger().debug(f"Mode: {mode}, Selected Joint: {joint}, Velocity: {self.velocity}")

    def movement_callback(self):
        """Timer callback to handle continuous movement"""
        if not self.active_keys:
            return

        for key in self.active_keys:
            if self.joint_mode:
                if key == keyboard.Key.up:
                    self.move_joint(self.selected_joint, self.velocity)
                elif key == keyboard.Key.down:
                    self.move_joint(self.selected_joint, -self.velocity)
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'

                if key == keyboard.Key.up:
                    msg.twist.linear.x = self.velocity
                elif key == keyboard.Key.down:
                    msg.twist.linear.x = -self.velocity
                elif key == keyboard.Key.left:
                    msg.twist.linear.y = self.velocity
                elif key == keyboard.Key.right:
                    msg.twist.linear.y = -self.velocity
                elif key == keyboard.Key.home:
                    msg.twist.linear.z = self.velocity
                elif key == keyboard.Key.end:
                    msg.twist.linear.z = -self.velocity

                if any([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]):
                    self.twist_pub.publish(msg)

    def cleanup_before_exit(self):
        """Call this before switching to MoveIt control"""
        self.emergency_stop()
        # Switch back to trajectory control mode if needed
        if not self.joint_mode:
            req = SetBool.Request()
            req.data = True  # True for joint trajectory mode
            self.mode_client.call_async(req)
            # Wait for mode switch to complete
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

def main():
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Configure terminal
        tty.setcbreak(sys.stdin.fileno())
        
        rclpy.init()
        node = KeyboardMoveSample()
        node.print_state()  # Initial state display
        
        # Create a thread for ROS spinning
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.daemon = True
        spin_thread.start()
        
        # Create a timer for status updates (every 0.5 seconds)
        def update_status():
            while rclpy.ok():
                node.print_state()
                time.sleep(0.5)
        
        # Start status update thread
        status_thread = threading.Thread(target=update_status)
        status_thread.daemon = True
        status_thread.start()
        
        try:
            # Keep the main thread alive and handle terminal input
            while rclpy.ok():
                # Check if terminal is active
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    char = sys.stdin.read(1)
                    if char == '\x03':  # Ctrl+C
                        break
                    # Handle terminal input here if needed
                    
        except KeyboardInterrupt:
            node.cleanup_before_exit()
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        os.system('clear')
        print("Roboter-Steuerung beendet.")

if __name__ == '__main__':
    main()
