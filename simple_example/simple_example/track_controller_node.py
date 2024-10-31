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
        self._mutex_cb_group = MutuallyExclusiveCallbackGroup() # Callbacks here cannot happen in parallel
        self._reentrant_cb_group = ReentrantCallbackGroup() # Callbacks here can happen in parallel

        # Publishers/Subscribers
        self._image_subscriber = self.create_subscription(
            Image, "/front_camera", self.imageDataCb, qos_profile=1, callback_group=self._mutex_cb_group) 
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timers
        self._cmd_vel_publish_rate = 1.0  # TODO: parametrize
        self._cmd_vel_publisher_timer = self.create_timer(
            1 / self._cmd_vel_publish_rate, self.publishCmdVelCb, callback_group=self._reentrant_cb_group)
        
        self._image_processing_rate = 0.5  # TODO: parametrize
        self._image_processing_timer = self.create_timer(
            1 / self._image_processing_rate, self.processImageCb, callback_group=self._mutex_cb_group)

        # Class fields
        self._current_frame = np.zeros((100,100))
        self._processed_frame = np.zeros((100,100))

        self._current_cmd_vel = Twist()
        self._cv_bridge = CvBridge()

        # Start the display thread
        self._display_thread = threading.Thread(target=self.display_image, daemon=True)
        self._display_thread.start()

    def imageDataCb(self, msg: Image) -> None:
        """
        This method will get the incoming Image message and convert it into an OpenCV image.
        """
        self.get_logger().info("Image data callback")
        try:
            self._current_frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def processImageCb(self):
        """
        This is a callback to the _image_processing_timer. It processes the image to update the cmd_vel data.
        """
        if self._current_frame is not None:
            self.get_logger().info("Processing image data")
            self._current_cmd_vel = self.processImage(self._current_frame)
        
    def publishCmdVelCb(self):
        """
        This callback publishes the latest cmd_vel data periodically.
        """
        self.get_logger().info("Publishing cmd_vel")
        self._cmd_vel_publisher.publish(self._current_cmd_vel)

    def display_image(self):
        """
        Continuously display images on a separate thread.
        """
        while rclpy.ok():
            if self._processed_frame is not None:
                cv2.imshow("Frame", self._processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on pressing 'q'
                    break
        cv2.destroyAllWindows()

    def region_of_interest(self, img, vertices):
      # Define a blank matrix that matches the image height/width.
      mask = np.zeros_like(img)
      # Retrieve the number of color channels of the image.
      channel_count = img.shape[2]
      # Create a match color with the same color channel counts.
      match_mask_color = (255,) * channel_count

      # Fill inside the polygon
      cv2.fillPoly(mask, vertices, match_mask_color)

      # Returning the image only where mask pixels match
      masked_image = cv2.bitwise_and(img, mask)

      return masked_image

    def processImage(self, img: np.ndarray) -> Twist:
      height, width,_ = img.shape
      region_of_interest_vertices = [
        (width * 0.1, height * 0.47),
        (0, height * 0.7),
        (width, height * 0.7),
        (width * 0.9 , height * 0.47),
        ]
      cropped = self.region_of_interest(img,np.array([region_of_interest_vertices], np.int32),)
      hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
      lower_yellow = np.array([25, 25, 25])  # Lower bound for yellow (adjust if needed)
      upper_yellow = np.array([255, 255, 255])  # Upper bound for yellow (adjust if needed)
      # Threshold the HSV image to get only yellow colors
      binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
      # Apply morphological operations to clean up the mask (optional)
      kernel = np.ones((5, 5), np.uint8)
      binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
      binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)


      self._processed_frame=binary_mask

      output_cmd_vel = Twist()
      output_cmd_vel.angular.x = 0.
      output_cmd_vel.angular.y = 0.
      output_cmd_vel.angular.z = 0.
      output_cmd_vel.linear.x = 0. # Just for testing
      output_cmd_vel.linear.y = 0.
      output_cmd_vel.linear.z = 0.
      return output_cmd_vel

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
