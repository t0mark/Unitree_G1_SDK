#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_hg.msg import LowState
import multiprocessing as mp


class SubscriberNode(Node):
    def __init__(self, topic, queue):
        super().__init__('bridge_subscriber')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.queue = queue
        self.subscription = self.create_subscription(
            LowState,
            topic,
            self.listener_callback,
            qos
        )
        self.get_logger().info(f'Subscriber started on domain {os.environ.get("ROS_DOMAIN_ID")} for topic {topic}')

    def listener_callback(self, msg):
        # Serialize message to bytes for queue transfer
        self.queue.put(msg)


class PublisherNode(Node):
    def __init__(self, topic, queue):
        super().__init__('bridge_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.queue = queue
        self.publisher = self.create_publisher(LowState, topic, qos)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info(f'Publisher started on domain {os.environ.get("ROS_DOMAIN_ID")} for topic {topic}')

    def timer_callback(self):
        while not self.queue.empty():
            try:
                msg = self.queue.get_nowait()
                self.publisher.publish(msg)
            except:
                break


def run_subscriber(from_domain, topic, queue):
    os.environ['ROS_DOMAIN_ID'] = str(from_domain)
    rclpy.init()
    node = SubscriberNode(topic, queue)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_publisher(to_domain, topic, queue):
    os.environ['ROS_DOMAIN_ID'] = str(to_domain)
    rclpy.init()
    node = PublisherNode(topic, queue)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = Node('domain_bridge_config')
    node.declare_parameter('from_domain', 0)
    node.declare_parameter('to_domain', 3)
    node.declare_parameter('topic', '/lowstate')

    from_domain = node.get_parameter('from_domain').value
    to_domain = node.get_parameter('to_domain').value
    topic = node.get_parameter('topic').value

    node.get_logger().info(f'Starting domain bridge: {topic} from domain {from_domain} to {to_domain}')
    node.destroy_node()
    rclpy.shutdown()

    # Create a queue for inter-process communication
    queue = mp.Queue(maxsize=100)

    # Start subscriber and publisher in separate processes
    sub_process = mp.Process(target=run_subscriber, args=(from_domain, topic, queue))
    pub_process = mp.Process(target=run_publisher, args=(to_domain, topic, queue))

    sub_process.start()
    pub_process.start()

    try:
        sub_process.join()
        pub_process.join()
    except KeyboardInterrupt:
        sub_process.terminate()
        pub_process.terminate()
        sub_process.join()
        pub_process.join()


if __name__ == '__main__':
    main()
