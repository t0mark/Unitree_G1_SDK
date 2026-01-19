#!/usr/bin/env python3
"""
Synchronize real G1 robot state to RViz visualization
Subscribes to /lowstate and /odommodestate topics and publishes TF/JointState for RViz
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState
from unitree_go.msg import SportModeState

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RvizSynchronizer(Node):
    """ROS2 node that synchronizes real robot state to RViz"""

    # G1 joint names matching URDF
    G1_JOINT_NAMES = [
        # Left leg (0-5)
        'left_hip_pitch_joint',
        'left_hip_roll_joint',
        'left_hip_yaw_joint',
        'left_knee_joint',
        'left_ankle_pitch_joint',
        'left_ankle_roll_joint',
        # Right leg (6-11)
        'right_hip_pitch_joint',
        'right_hip_roll_joint',
        'right_hip_yaw_joint',
        'right_knee_joint',
        'right_ankle_pitch_joint',
        'right_ankle_roll_joint',
        # Waist (12-14)
        'waist_yaw_joint',
        'waist_roll_joint',
        'waist_pitch_joint',
        # Left arm (15-21)
        'left_shoulder_pitch_joint',
        'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint',
        'left_elbow_joint',
        'left_wrist_roll_joint',
        'left_wrist_pitch_joint',
        'left_wrist_yaw_joint',
        # Right arm (22-28)
        'right_shoulder_pitch_joint',
        'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint',
        'right_elbow_joint',
        'right_wrist_roll_joint',
        'right_wrist_pitch_joint',
        'right_wrist_yaw_joint',
    ]

    def __init__(self):
        super().__init__('rviz_synchronizer')

        # Declare parameters
        self.declare_parameter('robot_model', 'g1')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('robot_frame', 'pelvis')
        self.declare_parameter('publish_rate', 50.0)

        # Get parameters
        self.robot_model = self.get_parameter('robot_model').value
        self.base_frame = self.get_parameter('base_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f'Initializing RViz synchronizer for {self.robot_model}')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Joint state publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # State storage
        self.motor_positions: List[float] = [0.0] * len(self.G1_JOINT_NAMES)
        self.motor_velocities: List[float] = [0.0] * len(self.G1_JOINT_NAMES)
        self.robot_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.robot_quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z

        # Subscribe to /lowstate
        self.state_sub = self.create_subscription(
            LowState,
            '/lowstate',
            self.state_callback,
            10
        )
        self.get_logger().info('Subscribed to /lowstate topic')

        # Subscribe to /odommodestate
        self.odom_sub = self.create_subscription(
            SportModeState,
            '/odommodestate',
            self.odom_callback,
            10
        )
        self.get_logger().info('Subscribed to /odommodestate topic')

        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_state)

        self.get_logger().info('RViz synchronizer initialized successfully')

    def odom_callback(self, msg: SportModeState):
        """Callback for /odommodestate topic"""
        self.robot_position = (msg.position[0], msg.position[1], msg.position[2])

    def state_callback(self, msg: LowState):
        """Callback for /lowstate topic"""
        # Extract motor positions and velocities
        num_joints = min(len(msg.motor_state), len(self.G1_JOINT_NAMES))
        for i in range(num_joints):
            self.motor_positions[i] = msg.motor_state[i].q
            self.motor_velocities[i] = msg.motor_state[i].dq

        # Extract IMU quaternion
        if len(msg.imu_state.quaternion) >= 4:
            self.robot_quaternion = (
                msg.imu_state.quaternion[0],  # w
                msg.imu_state.quaternion[1],  # x
                msg.imu_state.quaternion[2],  # y
                msg.imu_state.quaternion[3],  # z
            )

    def publish_state(self):
        """Publish TF transform and joint states"""
        now = self.get_clock().now().to_msg()

        # Publish TF transform (world -> pelvis)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.robot_frame

        t.transform.translation.x = self.robot_position[0]
        t.transform.translation.y = self.robot_position[1]
        t.transform.translation.z = self.robot_position[2]

        t.transform.rotation.w = self.robot_quaternion[0]
        t.transform.rotation.x = self.robot_quaternion[1]
        t.transform.rotation.y = self.robot_quaternion[2]
        t.transform.rotation.z = self.robot_quaternion[3]

        self.tf_broadcaster.sendTransform(t)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.G1_JOINT_NAMES
        joint_msg.position = self.motor_positions
        joint_msg.velocity = self.motor_velocities
        joint_msg.effort = []

        self.joint_pub.publish(joint_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = RvizSynchronizer()

        print("\n" + "="*70)
        print("RViz Synchronizer Running")
        print("="*70)
        print("\nPublishing to:")
        print("  - /joint_states (sensor_msgs/JointState)")
        print("  - /tf (geometry_msgs/TransformStamped)")
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
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
