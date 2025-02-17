import rclpy
from rclpy.node import Node

from realsense_object_center_msgs.msg import ObjectPoseDepthList


class ObjectPoseDepthSubscriber(Node):
    def __init__(self):
        super().__init__("object_pose_depth_subscriber")

        self.subscriber = self.create_subscription(
            ObjectPoseDepthList,
            "/object_pose_depth_list",
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(
            f"Received ObjectPoseDepthList with {len(msg.objects)} objects")
        for obj in msg.objects:
            self.get_logger().info(
                f"Object ID: {obj.id}, X: {obj.x}, Y: {obj.y}, Depth: {obj.d}")


def main():
    rclpy.init()

    # start console
    print("object_center_test_node start")

    object_pose_depth_subscriber = ObjectPoseDepthSubscriber()
    rclpy.spin(object_pose_depth_subscriber)

    # end console
    print("object_center_test_node end")

    object_pose_depth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
