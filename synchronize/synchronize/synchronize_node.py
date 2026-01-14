#!/usr/bin/env python3
"""
Synchronize real G1 robot state to Mujoco simulation
Subscribes to /lowstate topic and updates Mujoco visualization in real-time
"""

import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState
from unitree_go.msg import SportModeState

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("WARNING: mujoco module not found. Please install with: pip3 install mujoco")


class MujocoSynchronizer(Node):
    """ROS2 node that synchronizes real robot state to Mujoco simulation"""

    def __init__(self):
        super().__init__('mujoco_synchronizer')

        # Declare parameters
        self.declare_parameter('robot_model', 'g1')
        self.declare_parameter('scene_file',
            '/home/unitree/demo_ws/src/synchronize/models/g1/scene_empty.xml',)
        self.declare_parameter('viewer_fps', 50.0)

        # Camera follow parameters
        self.declare_parameter('camera_follow', True)
        self.declare_parameter('camera_distance', 3.0)
        self.declare_parameter('camera_azimuth', 90.0)
        self.declare_parameter('camera_elevation', -20.0)
        self.declare_parameter('camera_lookat_offset_x', 0.0)
        self.declare_parameter('camera_lookat_offset_y', 0.0)
        self.declare_parameter('camera_lookat_offset_z', 0.5)

        # Get parameters
        self.robot_model = self.get_parameter('robot_model').value
        self.scene_file = self.get_parameter('scene_file').value
        self.viewer_fps = self.get_parameter('viewer_fps').value

        # Camera parameters
        self.camera_follow = self.get_parameter('camera_follow').value
        self.camera_distance = self.get_parameter('camera_distance').value
        self.camera_azimuth = self.get_parameter('camera_azimuth').value
        self.camera_elevation = self.get_parameter('camera_elevation').value
        self.camera_lookat_offset = [
            self.get_parameter('camera_lookat_offset_x').value,
            self.get_parameter('camera_lookat_offset_y').value,
            self.get_parameter('camera_lookat_offset_z').value,
        ]

        self.get_logger().info(f'Initializing Mujoco simulation for {self.robot_model}')
        self.get_logger().info(f'Loading scene: {self.scene_file}')

        if not MUJOCO_AVAILABLE:
            self.get_logger().error('Mujoco is not installed!')
            self.get_logger().error('Install with: pip3 install mujoco')
            raise RuntimeError('Mujoco not available')

        # Check if scene file exists
        if not Path(self.scene_file).exists():
            self.get_logger().error(f'Scene file not found: {self.scene_file}')
            raise FileNotFoundError(f'Scene file not found: {self.scene_file}')

        # Initialize Mujoco
        try:
            self.mj_model = mujoco.MjModel.from_xml_path(self.scene_file)
            self.mj_data = mujoco.MjData(self.mj_model)
        except Exception as e:
            self.get_logger().error(f'Failed to load Mujoco model: {e}')
            raise

        self.get_logger().info(f'Mujoco model loaded: {self.mj_model.nv} DOF')

        # Thread synchronization
        self.locker = threading.Lock()
        self.running = True

        # Statistics
        self.msg_count = 0
        self.last_msg_time = self.get_clock().now()
        self.last_100_msg_time = self.get_clock().now()

        # Subscribe to /lowstate
        self.state_sub = self.create_subscription(
            LowState,
            '/lowstate',
            self.state_callback,
            10
        )

        self.get_logger().info('Subscribed to /lowstate topic')

        # Subscribe to /odommodestate for robot position
        self.odom_sub = self.create_subscription(
            SportModeState,
            '/odommodestate',
            self.odom_callback,
            10
        )

        self.get_logger().info('Subscribed to /odommodestate topic')

        # Launch Mujoco viewer in passive mode
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)

        # Set initial camera position
        self.viewer.cam.distance = self.camera_distance
        self.viewer.cam.azimuth = self.camera_azimuth
        self.viewer.cam.elevation = self.camera_elevation

        self.get_logger().info('Mujoco viewer launched')
        self.get_logger().info(f'Camera follow: {self.camera_follow}')

        # Start viewer thread
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.viewer_thread.start()

        self.get_logger().info('Synchronizer initialized successfully')

    def odom_callback(self, msg: SportModeState):
        """
        Callback for /odommodestate topic
        Updates Mujoco simulation base position from odometry
        """
        with self.locker:
            if not self.mj_model or not self.mj_data:
                return

            # Update floating base position (first 3 elements of qpos)
            # SportModeState contains position information
            self.mj_data.qpos[0] = msg.position[0]  # x position
            self.mj_data.qpos[1] = msg.position[1]  # y position
            self.mj_data.qpos[2] = msg.position[2]  # z position

            # Update floating base linear velocity (first 3 elements of qvel)
            self.mj_data.qvel[0] = msg.velocity[0]  # x velocity
            self.mj_data.qvel[1] = msg.velocity[1]  # y velocity
            self.mj_data.qvel[2] = msg.velocity[2]  # z velocity

            # Forward kinematics to update visualization
            mujoco.mj_forward(self.mj_model, self.mj_data)

    def state_callback(self, msg: LowState):
        """
        Callback for /lowstate topic
        Updates Mujoco simulation state from real robot
        """
        with self.locker:
            if not self.mj_model or not self.mj_data:
                return

            # Update joint positions and velocities
            # G1 robot has up to 35 motors (unitree_hg IDL)
            # but typically uses 23 or 29 DOF depending on configuration
            num_motors = min(len(msg.motor_state), self.mj_model.nu)

            for i in range(num_motors):
                motor_state = msg.motor_state[i]

                # Update joint position (qpos)
                # First 7 elements of qpos: 3 position + 4 quaternion (floating base)
                # Remaining elements: joint positions
                qpos_idx = i + 7  # Skip floating base
                if qpos_idx < self.mj_model.nq:
                    self.mj_data.qpos[qpos_idx] = motor_state.q

                # Update joint velocity (qvel)
                # First 6 elements of qvel: 3 linear + 3 angular velocity (floating base)
                # Remaining elements: joint velocities
                qvel_idx = i + 6  # Skip floating base velocities
                if qvel_idx < self.mj_model.nv:
                    self.mj_data.qvel[qvel_idx] = motor_state.dq

            # Update IMU orientation (quaternion)
            # Mujoco uses [w, x, y, z] format
            if len(msg.imu_state.quaternion) >= 4:
                # IMU sends [w, x, y, z] format (same as Mujoco)
                self.mj_data.qpos[3] = msg.imu_state.quaternion[0]  # w
                self.mj_data.qpos[4] = msg.imu_state.quaternion[1]  # x
                self.mj_data.qpos[5] = msg.imu_state.quaternion[2]  # y
                self.mj_data.qpos[6] = msg.imu_state.quaternion[3]  # z

            # Forward kinematics to update visualization
            mujoco.mj_forward(self.mj_model, self.mj_data)

        # Update statistics
        self.msg_count += 1
        current_time = self.get_clock().now()

        if self.msg_count % 100 == 0:
            duration = (current_time - self.last_100_msg_time).nanoseconds / 1e9
            rate = 100.0 / duration if duration > 0 else 0
            self.get_logger().info(
                f'Received {self.msg_count} states, rate: {rate:.1f} Hz'
            )
            self.last_100_msg_time = current_time

    def viewer_loop(self):
        """
        Viewer thread loop
        Renders Mujoco simulation at specified FPS
        """
        self.get_logger().info(f'Starting viewer loop at {self.viewer_fps:.1f} FPS')

        frame_duration = 1.0 / self.viewer_fps

        while self.running and self.viewer.is_running():
            start_time = time.perf_counter()

            # Sync viewer with simulation data
            with self.locker:
                # Update camera lookat to follow robot
                if self.camera_follow:
                    robot_pos = self.mj_data.qpos[0:3]  # x, y, z
                    self.viewer.cam.lookat[0] = robot_pos[0] + self.camera_lookat_offset[0]
                    self.viewer.cam.lookat[1] = robot_pos[1] + self.camera_lookat_offset[1]
                    self.viewer.cam.lookat[2] = robot_pos[2] + self.camera_lookat_offset[2]

                self.viewer.sync()

            # Rate control
            elapsed = time.perf_counter() - start_time
            if elapsed < frame_duration:
                time.sleep(frame_duration - elapsed)

        self.get_logger().info('Viewer loop terminated')

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down synchronizer')
        self.running = False
        if hasattr(self, 'viewer_thread'):
            self.viewer_thread.join(timeout=2.0)


def main(args=None):
    """Main entry point"""

    # Check if mujoco is available
    if not MUJOCO_AVAILABLE:
        print("\n" + "="*70)
        print("ERROR: Mujoco Python package is not installed!")
        print("="*70)
        print("\nPlease install mujoco with one of these methods:\n")
        print("1. Using pip:")
        print("   pip3 install mujoco")
        print("\n2. Using pip with user flag:")
        print("   pip3 install --user mujoco")
        print("\n3. In a virtual environment:")
        print("   python3 -m venv ~/mujoco_env")
        print("   source ~/mujoco_env/bin/activate")
        print("   pip install mujoco")
        print("\n" + "="*70 + "\n")
        return 1

    rclpy.init(args=args)

    try:
        node = MujocoSynchronizer()

        print("\n" + "="*70)
        print("Mujoco Synchronizer Running")
        print("="*70)
        print("\nWaiting for /lowstate messages from real robot...")
        print("Press Ctrl+C to exit\n")

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nShutdown requested by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
