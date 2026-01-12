# Synchronize - Real Robot to Mujoco Simulation

ROS2 package for synchronizing real Unitree robot movements to Mujoco simulation visualization.

## Overview

This package subscribes to the `/lowstate` topic from a real Unitree robot (G1, H1, Go2, etc.) and displays the robot's movements in a Mujoco simulation window in real-time. This is the inverse of sim-to-real: **real-to-sim** for visualization and debugging purposes.

## Features

- Real-time synchronization of robot joint positions and velocities
- IMU orientation visualization
- 50 FPS smooth rendering
- Support for multiple Unitree robot models (G1, H1, Go2, B2, etc.)
- Configurable scene files
- Statistics monitoring (message rate, etc.)
- **Python-based** for easy installation and modification

## Dependencies

### System Dependencies
- ROS2 (Humble or later)
- Unitree SDK2 with ROS2 messages (`unitree_go`)
- Python 3.8+

### Python Dependencies
- `mujoco` - Mujoco physics simulator Python bindings
- `rclpy` - ROS2 Python client library

## Installation

### 1. Install Mujoco Python Package

```bash
# Option 1: System-wide installation
pip3 install mujoco

# Option 2: User installation (recommended if you don't have sudo)
pip3 install --user mujoco

# Option 3: Using virtual environment
python3 -m venv ~/mujoco_env
source ~/mujoco_env/bin/activate
pip install mujoco
```

### 2. Build the ROS2 Package

```bash
cd ~/demo_ws
colcon build --packages-select synchronize
source install/setup.bash
```

### 3. Verify Installation

```bash
# Test if mujoco is available
python3 -c "import mujoco; print(f'Mujoco {mujoco.__version__} installed')"

# Check ROS2 package
ros2 pkg list | grep synchronize
```

## Usage

### Basic Usage (G1 Robot)

1. Make sure your real G1 robot is running and publishing to `/lowstate` topic:
```bash
# Check if topic is available
ros2 topic list | grep lowstate
ros2 topic hz /lowstate
```

2. Launch the synchronizer:
```bash
ros2 launch synchronize synchronize.launch.py
```

3. You should see a Mujoco window showing your robot's real-time movements

### Custom Robot Model

For H1 robot:
```bash
ros2 launch synchronize synchronize.launch.py \
    robot_model:=h1 \
    scene_file:=/home/unitree/demo_ws/src/unitree_mujoco/unitree_robots/h1/scene.xml
```

For Go2 robot:
```bash
ros2 launch synchronize synchronize.launch.py \
    robot_model:=go2 \
    scene_file:=/home/unitree/demo_ws/src/unitree_mujoco/unitree_robots/go2/scene.xml
```

## Troubleshooting

### "Mujoco module not found" error

If you see this error:
```
WARNING: mujoco module not found. Please install with: pip3 install mujoco
```

**Solution:**
```bash
pip3 install mujoco
# or
pip3 install --user mujoco
```

### "Could not load model" error

**Solution:**
- Check that `scene_file` parameter points to a valid XML file
- Verify the path: `ls -l /path/to/scene.xml`

### No visualization window appears

**Check display:**
```bash
echo $DISPLAY
```

**Try software rendering:**
```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch synchronize synchronize.launch.py
```

### Robot not moving in simulation

**Verify topic:**
```bash
ros2 topic hz /lowstate
ros2 topic echo /lowstate --once
```

## File Structure

```
synchronize/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS2 package manifest
├── README.md                   # This file
├── scripts/
│   └── synchronize_node.py     # Main Python node
├── synchronize/
│   └── __init__.py             # Python package marker
└── launch/
    └── synchronize.launch.py   # Launch file
```

## License

MIT License
