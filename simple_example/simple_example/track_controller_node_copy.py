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

        # Callback groups
        self._mutex_cb_group = MutuallyExclusiveCallbackGroup()
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Publishers/Subscribers
        self._image_subscriber = self.create_subscription(
            Image, "/front_camera", self.imageDataCb, qos_profile=1, callback_group=self._mutex_cb_group) 
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timers
        self._cmd_vel_publish_rate = 20.0  # TODO: parametrize
        self._cmd_vel_publisher_timer = self.create_timer(
            1 / self._cmd_vel_publish_rate, self.publishCmdVelCb, callback_group=self._reentrant_cb_group)
        
        self._image_processing_rate = 20.0  # TODO: parametrize
        self._image_processing_timer = self.create_timer(
            1 / self._image_processing_rate, self.processImageCb, callback_group=self._mutex_cb_group)

        # Class fields
        self._current_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        self._processed_frame = np.zeros((100, 100), dtype=np.uint8)
        self._line_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        
        self._current_cmd_vel = Twist()
        self._cv_bridge = CvBridge()

        # Start the display thread
        self._display_thread = threading.Thread(target=self.display_image, daemon=True)
        self._display_thread.start()

    def imageDataCb(self, msg: Image) -> None:
        """Converts incoming Image message to an OpenCV image."""
        try:
            self._current_frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def processImageCb(self):
        """Processes the image to update cmd_vel data."""
        if self._current_frame is not None:
            self._current_cmd_vel, self._line_frame = self.processImage(self._current_frame)
        
    def publishCmdVelCb(self):
        """Publishes the latest cmd_vel data periodically."""
        self._cmd_vel_publisher.publish(self._current_cmd_vel)

    def display_image(self):
        """Displays the original frame, processed frame, and line frame with centroids."""
        while rclpy.ok():
            if self._current_frame is not None:
                cv2.imshow("Original Frame", self._current_frame)
            if self._line_frame is not None:
                cv2.imshow("Line with Centroids", self._line_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def region_of_interest(self, img, vertices):
        """Masks the input image to only show a region of interest."""
        mask = np.zeros_like(img)
        match_mask_color = (255,) * img.shape[2]
        cv2.fillPoly(mask, vertices, match_mask_color)
        return cv2.bitwise_and(img, mask)

    def processImage(self, img: np.ndarray) -> tuple:
        """Processes the input image to detect two lines and output command velocity based on their midpoint."""
        height, width, _ = img.shape

        # Define the rectangular region for line detection
        rect_height = 150
        rect_top = height // 2 - rect_height // 2 + 70
        rect_bottom = rect_top + rect_height

        # Extract only the region of interest (rectangle)
        rect_img = img[rect_top:rect_bottom, 0:width]

        # Blur
        rect_img = cv2.GaussianBlur(rect_img, (5, 5), 0)

        # Convert to HSV and filter color in the rectangle
        hsv = cv2.cvtColor(rect_img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 25, 25])
        upper_yellow = np.array([30, 255, 255])
        binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

        # Calculate line positions using Hough Transform
        lines = cv2.HoughLinesP(binary_mask, 1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=5)

        line_frame = img.copy()
        output_cmd_vel = Twist()

        # Draw the rectangle on line_frame
        cv2.rectangle(line_frame, (0, rect_top), (width, rect_bottom), (255, 0, 0), 2)

        if lines is not None:
            # Find the leftmost and rightmost line positions
            left_x = float('inf')
            right_x = float('-inf')

            for line in lines:
                for x1, y1, x2, y2 in line:
                    left_x = min(left_x, x1, x2)
                    right_x = max(right_x, x1, x2)

            if left_x < float('inf') and right_x > float('-inf'):
                # Calculate the midpoint
                cx_full = (left_x + right_x) // 2
                cy_full = (rect_top + rect_bottom) // 2

                # Draw the centroid and detected line area
                cv2.circle(line_frame, (cx_full, cy_full), 5, (0, 255, 0), -1)
                line_frame[rect_top:rect_bottom, :][binary_mask > 0] = (0, 255, 255)

                # Calculate deviation and set linear velocity
                deviation = cx_full - (width / 2)

                # Adjust linear speed based on deviation
                max_linear_speed = 10.0
                min_linear_speed = 5.0
                output_cmd_vel.linear.x = max(min_linear_speed, max_linear_speed - abs(deviation) / (width / 2) * (max_linear_speed - min_linear_speed))

                # Adjust angular control based on deviation
                angular_coefficient = -0.01  # Lower this value for a smoother turn
                target_angular_z = angular_coefficient * deviation

                # Smoothing transition of angular velocity
                if hasattr(self, '_last_angular_z'):
                    output_cmd_vel.angular.z = 0.8 * self._last_angular_z + 0.2 * target_angular_z
                else:
                    output_cmd_vel.angular.z = target_angular_z  # First time setting

                self._last_angular_z = output_cmd_vel.angular.z

                # Optional: Add a limit on angular velocity to avoid too sharp turns
                output_cmd_vel.angular.z = max(min(output_cmd_vel.angular.z, 1.0), -1.0)  # Clamp to [-1, 1]

                # Draw deviation and command velocities on the line frame
                cv2.putText(line_frame, f"Deviation: {deviation:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(line_frame, f"Linear Vel: {output_cmd_vel.linear.x:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(line_frame, f"Angular Vel: {output_cmd_vel.angular.z:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                output_cmd_vel.linear.x = 0.0
                output_cmd_vel.angular.z = 0.0
        else:
            output_cmd_vel.linear.x = 0.0
            output_cmd_vel.angular.z = 0.0

        return output_cmd_vel, line_frame




def main(args=None):
    rclpy.init(args=args)
    track_controller = TrackController()
    multithread_executor = MultiThreadedExecutor()
    multithread_executor.add_node(track_controller)
    multithread_executor.spin()
    track_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()