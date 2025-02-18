# pip install pyrealsense2==2.54.1.5217
import pyrealsense2 as rs
import numpy as np
import cv2

REAL_TIME = True  # True이면 실시간 모드, False이면 단일 프레임 모드


def gaussian_blur(cv_image):
    blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
    return blurred


def color_to_gray(cv_image):
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    return gray


def image_to_binary(cv_image):
    # _, binary = cv2.threshold(
    #     cv_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, binary = cv2.threshold(
        cv_image, 127, 255, cv2.THRESH_BINARY_INV)
    return binary


def closing(binary):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return closed


def find_contours(binary):
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def get_center(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy


def draw_center(cv_image, center):
    if center is not None:
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
    return cv_image


class Result:
    def __init__(self, id, x, y, depth):
        self.id = id
        self.x = x
        self.y = y
        self.depth = depth


def process_frame(color_image, depth_frame):
    gray = color_to_gray(color_image)
    blurred = gaussian_blur(gray)
    binary = image_to_binary(blurred)
    closed = closing(binary)
    contours = find_contours(closed)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    results = []

    for i, contour in enumerate(contours):
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)

        cv2.drawContours(color_image, [approx], 0, (0, 255, 0), 2)
        center = get_center(approx)
        color_image = draw_center(color_image, center)

        if center is not None:
            depth = depth_frame.get_distance(center[0], center[1])
            result = Result(id=i+1, x=center[0], y=center[1], depth=depth)
            results.append(result)

            cv2.putText(color_image, f"id: {result.id}", (result.x+10, result.y), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(color_image, f"{result.depth:.2f}m", (result.x+10, result.y+30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 2, cv2.LINE_AA)

    return color_image, results


if __name__ == '__main__':
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Align objects
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start streaming
    pipeline.start(config)

    try:
        if REAL_TIME:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())

                result_image, results = process_frame(color_image, depth_frame)
                cv2.imshow("Result", result_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        else:
            # Single frame processing
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if depth_frame and color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                result_image, results = process_frame(color_image, depth_frame)
                cv2.imshow("Result", result_image)
                cv2.waitKey(0)

    finally:
        # 스트리밍 종료
        pipeline.stop()
        cv2.destroyAllWindows()
