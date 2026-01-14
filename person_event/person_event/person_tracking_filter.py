import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray

PERSON_ID = 0
PERSON_NAME = "person"
MIN_SCORE = 0.0  # 필요하면 0.7로 올려

class PersonTrackingFilter(Node):
    def __init__(self):
        super().__init__('person_tracking_filter')
        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            DetectionArray,
            '/yolo/person_tracking',
            10
        )

    def cb(self, msg: DetectionArray):
        out = DetectionArray()
        out.header = msg.header
        out.detections = [
            d for d in msg.detections
            if ((d.class_id == PERSON_ID) or (d.class_name == PERSON_NAME))
            and d.score >= MIN_SCORE
        ]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = PersonTrackingFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()