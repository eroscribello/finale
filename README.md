# Final project Robotics Lab


## 0. Build and source
Before doing anything you need to build...
```
colcon build
```
...and source in every terminal:
```
source install/setup.sh
```

## 1. Simulation Environment
To run the full simulation, launch the following components in separate terminals:
```
ros2 launch robot_sim multi_robot.launch.py 
```

## 2. Vision & Arm Control (Action Server)
Start the arm controller in vision mode. This node hosts the Action Server managing Inverse Kinematics (KDL) and ArUco tracking:
```
ros2 launch ros2_kdl_package launching.launch.py ctrl:=vision
```

##3. Navigation Stack (Nav2)
Initialize the navigation servers (Planner, Controller, Recovery) for the Fra2Mo base:
```
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

##4. Task Supervisor (The Client)
Execute the main node that triggers the entire mission sequence:
```
ros2 run ros2_kdl_package ros2_kdl_node_client 
```
