import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from realsense_object_center_msgs.msg import ObjectPoseDepth, ObjectPoseDepthList

import cv2
import queue


class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__("realsense_subscriber")

        self.sub_color = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.callback_color,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.callback_depth,
            10
        )

        self.bridge = CvBridge()

        self.object_pose_depth_pub = self.create_publisher(
            ObjectPoseDepthList,
            "/object_pose_depth_list",
            10
        )

        self.queue = queue.Queue()

    def callback_color(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)

        gray = self.color_to_gray(cv_image)
        bit_not = cv2.bitwise_not(gray)
        blurred = self.gaussian_blur(bit_not)
        binary = self.image_to_binary(blurred)
        closed = self.closing(binary)
        cv2.imshow("closed", closed)
        contours = self.find_contours(closed)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        result = []
        for i, contour in enumerate(contours):
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)

            cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 2)
            center = self.get_center(approx)
            cv_image = self.draw_center(cv_image, center)
            cv2.putText(cv_image, str(i+1), center, cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 2, cv2.LINE_AA)
            result.append(center)

        self.queue.put(result)
        cv2.imshow("color", cv_image)
        cv2.waitKey(1)

    def callback_depth(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            print(e)

        object_pose_depth_list = ObjectPoseDepthList()

        while not self.queue.empty():
            centers = self.queue.get()
            print(f"founded {len(centers)} objects")
            for i, center in enumerate(centers):
                if center is not None:
                    x, y = center
                    depth = cv_image[y, x]

                    object_pose_depth = ObjectPoseDepth()
                    object_pose_depth.id = i+1
                    object_pose_depth.x = x
                    object_pose_depth.y = y
                    object_pose_depth.d = depth.item()

                    object_pose_depth_list.objects.append(object_pose_depth)

                    print(f"object {i+1} depth: {depth}")

        self.object_pose_depth_pub.publish(object_pose_depth_list)

    def gaussian_blur(self, cv_image):
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        return blurred

    def color_to_gray(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        return gray

    def image_to_binary(self, cv_image):
        _, binary = cv2.threshold(
            cv_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary

    def closing(self, binary):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        return closed

    def find_contours(self, binary):
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def get_center(self, contour):
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    def draw_center(self, cv_image, center):
        if center is not None:
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        return cv_image

    def pixel_to_depth(self, x, y, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        return cv_image[y, x]


def main():
    rclpy.init()

    # start console
    print("realsense_object_center_node start")

    realsense_subscriber = RealsenseSubscriber()
    rclpy.spin(realsense_subscriber)

    # end console
    print("realsense_object_center_node end")

    realsense_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
