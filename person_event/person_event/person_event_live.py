import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray

PERSON_CLASS_ID = 0
PERSON_CLASS_NAME = "person"

class PersonEvent(Node):
    def __init__(self):
        super().__init__('person_event_live')

        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/person_detected',
            10
        )

    def cb(self, msg: DetectionArray):
        person_now = any(
            (d.class_id == PERSON_CLASS_ID) or (d.class_name == PERSON_CLASS_NAME)
            for d in msg.detections
        )

        if person_now:
            out = String()
            out.data = "person_detected"
            self.pub.publish(out)
            self.get_logger().info("👤 person_detected")

def main():
    rclpy.init()
    node = PersonEvent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()