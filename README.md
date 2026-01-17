
# Panda Pick and Place - ROS 2 & MoveIt 2
### Robot Control Systems Academic year 2025-2026, Vlad Ciomîrtan
This package implements robotic manipulation tasks using the **Franka Emika Panda** robot. It leverages **MoveIt 2** for motion planning and **ROS 2 Humble** for node communication.

### 🚀 Features
- **Simple Pick & Place**: Pick a single cube and lift it to a target height.
- **Cube Stacking**: A complex task where the robot identifies, picks, and stacks three cubes vertically.
- **Automated Cleanup**: The node resets the planning scene and robot pose on every start.

### 📁 Package Structure
    ├── README.md
    └── panda_pick_place
        ├── CMakeLists.txt
        ├── include
        │   └── panda_pick_place
        ├── launch
        │   ├── pick_place.launch.py
        │   └── stacking.launch.py
        ├── package.xml
        └── src
            ├── pick_place_node.cpp
            └── stacking_cubes_node.cpp
---
### 🛠️ Setup & Installation

#### Prerequisites
- **ROS 2 Humble**
- **MoveIt 2**
- **Panda MoveIt Config**: `sudo apt install ros-humble-moveit-resources-panda-moveit-config`

#### Build Instructions
Run from the root of your workspace:
```bash
cd ~/panda_ws
colcon build --packages-select panda_pick_place
source install/setup.bash
```

#### To run the simulation
Open the first terminal (environment) and launch the RViz simulation and MoveGroup interface:
```bash
source ~/panda_ws/install/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
Open a second terminal and run the simple pick and lift task:
```bash
ros2 launch panda_pick_place pick_place.launch.py
```

Or the more advanced cube stacking task:
```bash
ros2 launch panda_pick_place stacking.launch.py
```
