# igus_ws - Einführung in die Robotik mit dem Igus-Rebel und ROS2

## Voraussetzungen 

- ROS2 Jazzy Jalisco
- Ubuntu 24.04. LTS

## Installation

```bash
git clone --recurse-submodules https://github.com/sibeschu/igus_ws.git
```

dann

```bash
cd igus_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

Die IP des Netzwerkadapters, der für die Verbindung mit dem Roboter genutzt wird muss sich im Netzwerk von 192.168.3.1 befinden.  
Diese wird festgelegt in `igus_rebel_ros2/src/igus_rebel/include/Rebel.hpp` (Zeile 88)

## Einstiegspunkte

Workspace "sourcen" vom root-Ordner:  

`source install/setup.bash`

Starten der Simulation :  

```bash
ros2 launch igus_rebel_moveit_config igus_rebel_simulated.launch.py
```

```bash
src/
├── igus_student
│   ├── igus_student
│   │   ├── __init__.py
│   │   └── simple_robot_control.py
```
In `simple_robot_control.py` können Anweisungen für den Igus geschrieben werden.  
Mit `colcon build --packages-select igus_student` im root-Ordner den Workspace erneut bauen, um Änderungen zu übernehmen.  
Interface für den Igus mit `ros2 launch igus_rebel rebel.launch.py` staren.  
MotionPlanningPipeline mit `ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true`starten.  
Ausführen mit `ros2 run igus_student student_robot_control`  

```bash
src
├── sample_package
│   ├── sample_package
│   │   └── keyboard_move_sample.py
```
Mit dem `keyboard_move_sample.py` kann erkundet werden, wie sich der Roboter über /delta_joint_cmd und /delta_twist/cmd steuern lässt.  
- Interface starten mit `ros2 launch igus_rebel rebel.launch.py`
- MotionPlanningPipeline starten mit `ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py`
- `ros2 run sample_package keyboard_move_sample` startet das Programm 

## Dokumentationen und Sourcecode

- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- [igus_rebel_ros2](https://bitbucket.org/truphysics/igus_rebel_ros2/src/main/)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/bafc21080c5c8e259dadbb309797949aee0dd950)
- [yolo_ros](https://github.com/mgonzs13/yolo_ros/tree/ecdc718c8600b0c0744e32477cd121de55be5e30)

## Bekannte Probleme

- Motionplanner funktioniert nicht --> locales auf en_US ? 
- Workspace nicht gesourced --> gut prüfbar mit `echo $ROS_DISTRO` (zeigt nicht was genau gesourced ist)
- wenn Roboter bei Anweisung nur kurz zuckt und dann in den roten Error-Modus geht --> Roboter neu starten am Knopf

## NOT-AUS

Nach Betätigen des NOT-Aus : 
- NOT-Aus rausdrehen -> NOT-Aus deaktivieren
- Robot Interface beenden mit `STRG + C`
- Robot Interface neu starten mit `ros2 launch igus_rebel rebel.launch.py`
- in Rviz eine sichere Position anfahren über den MotionPlanner
  - Goal_State per Grafikoberfläche setzen oder Goal_State "home" auswählen und dann Plan & Execute 
