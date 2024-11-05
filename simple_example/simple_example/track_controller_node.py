import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class TrackController(Node):
    def __init__(self):
        super().__init__('track_controller')

        # Callback groups for threading
        self._mutex_cb_group = MutuallyExclusiveCallbackGroup()
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Publishers/Subscribers
        self._image_subscriber = self.create_subscription(
            Image, "/front_camera", self.imageDataCb, qos_profile=1, callback_group=self._mutex_cb_group) 
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timers for publishing and processing rates
        self._cmd_vel_publish_rate = 40.0
        self._cmd_vel_publisher_timer = self.create_timer(
            1 / self._cmd_vel_publish_rate, self.publishCmdVelCb, callback_group=self._reentrant_cb_group)
        
        self._image_processing_rate = 40.0
        self._image_processing_timer = self.create_timer(
            1 / self._image_processing_rate, self.processImageCb, callback_group=self._mutex_cb_group)

        # Class fields for image and command storage
        self._current_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        self._line_frame = np.zeros((100, 100, 3), dtype=np.uint8)

        self._current_cmd_vel = Twist()
        self._cv_bridge = CvBridge()

        # Start the display thread
        self._display_thread = threading.Thread(target=self.display_image, daemon=True)
        self._display_thread.start()

        # Hold the last known command velocities
        self._last_cmd_vel = Twist()

    def imageDataCb(self, msg: Image) -> None:
        try:
            self._current_frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def processImageCb(self):
        if self._current_frame is not None:
            self._current_cmd_vel, self._line_frame = self.processImage(self._current_frame)
        
    def publishCmdVelCb(self):
        """Publishes the latest cmd_vel data periodically."""
        self._cmd_vel_publisher.publish(self._current_cmd_vel)

    def display_image(self):
        while rclpy.ok():
            if self._current_frame is not None:
                cv2.imshow("Original Frame", self._current_frame)
            if self._line_frame is not None:
                cv2.imshow("Line with Centroids", self._line_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def processImage(self, img: np.ndarray) -> tuple:
        height, width, _ = img.shape

        # Define maximum linear speed
        max_linear_speed = 8.0
        min_linear_speed = 0.5  

        # Define rectangular region for line detection
        rect_height = 90
        rect_top = height // 2 - rect_height // 2 +80
        rect_bottom = rect_top + rect_height

        # Extract region of interest
        rect_img = img[rect_top:rect_bottom, 0:width]

        # Blur
        rect_img = cv2.GaussianBlur(rect_img, (5, 5), 0)

        # Convert to HSV and color filter
        hsv = cv2.cvtColor(rect_img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 25, 25])
        upper_yellow = np.array([30, 255, 255])
        binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Clean the mask
        kernel = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

        # Detect lines
        lines = cv2.HoughLinesP(binary_mask, 1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=5)

        line_frame = img.copy()
        output_cmd_vel = Twist()

        # Draw rectangle
        cv2.rectangle(line_frame, (0, rect_top), (width, rect_bottom), (255, 0, 0), 2)

        # Initialize left and right positions
        left_xs = []
        right_xs = []

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x1 < width // 2:
                        left_xs.append(x1)
                        left_xs.append(x2)
                    else:
                        right_xs.append(x1)
                        right_xs.append(x2)

            left_x = np.mean(left_xs) if left_xs else float('inf')
            right_x = np.mean(right_xs) if right_xs else float('-inf')

            if left_x < float('inf') and right_x > float('-inf'):
                cx_full = (left_x + right_x) // 2
                cy_full = (rect_top + rect_bottom) // 2

                cv2.circle(line_frame, (int(cx_full), int(cy_full)), 5, (0, 255, 0), -1)
                cv2.line(line_frame, (int(left_x), rect_top), (int(left_x), rect_bottom), (255, 0, 255), 2)
                cv2.line(line_frame, (int(right_x), rect_top), (int(right_x), rect_bottom), (255, 0, 255), 2)
                line_frame[rect_top:rect_bottom, :][binary_mask > 0] = (0, 255, 255)

                deviation = cx_full - (width / 2)
                angular_speed_factor = 10.0  
                self._last_cmd_vel.angular.z = -deviation / (width / 2) * angular_speed_factor

                output_cmd_vel.linear.x = max(min_linear_speed, max_linear_speed)
                output_cmd_vel.angular.z = max(-10.0, min(10.0, self._last_cmd_vel.angular.z))  # Limit angular speed to ±10.0

                # Display information on screen
                cv2.putText(line_frame, f"Deviation: {deviation:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(line_frame, f"Linear Speed: {output_cmd_vel.linear.x:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(line_frame, f"Angular Speed: {output_cmd_vel.angular.z:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            else:
                # Line lost; maintain linear speed
                output_cmd_vel.linear.x = max_linear_speed
                output_cmd_vel.angular.z = 0.0

        return output_cmd_vel, line_frame



def main(args=None):
    rclpy.init(args=args)
    track_controller = TrackController()
    executor = MultiThreadedExecutor()
    rclpy.spin(track_controller, executor)
    track_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
