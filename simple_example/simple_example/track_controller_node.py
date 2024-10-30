import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .process_image import processImage
class TrackController(Node):
    def __init__(self):
        super().__init__('track_controller')

        # Callback groups
        self._mutex_cb_group = MutuallyExclusiveCallbackGroup() # Callbacks here cannot happen in parallel
        self._reentrant_cb_group = ReentrantCallbackGroup() # Callbacks here cannot happen in parallel

        # Publishers/Subscribers
        self._image_subscriber = self.create_subscription(Image,"/front_camera", self.imageDataCb,qos_profile=1, callback_group=self._mutex_cb_group) 
        self._cmd_vel_publisher = self.create_publisher(Twist,"/cmd_vel",10)

        # Timers
        self._cmd_vel_publish_rate = 1.0 
        self._cmd_vel_publisher_timer = self.create_timer(1 / self._cmd_vel_publish_rate, self.publishCmdVelCb, callback_group=self._reentrant_cb_group)

        self._image_processing_rate = 1.0 
        self._image_processing_timer = self.create_timer(1 / self._image_processing_rate, self.processImageCb, callback_group=self._mutex_cb_group)


        # Class fields
        self._current_frame = None
        self._current_cmd_vel = Twist()
        self._cv_bridge = CvBridge()
 
    def imageDataCb(self, msg:Image) -> None:
      """
      This method will get the incoming Image message and convert it into opencv image
      """
      self.get_logger().info("Image data callback")
      try:
        self._current_frame = self._cv_bridge.imgmsg_to_cv2(msg,"bgr8") # This may need changing. See https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html for message definition
      except Exception as e:
        self.get_logger().error(f"Failed to convert image: {e}")
        return

  
    def processImageCb(self):
      """
      This is a callback to the _image_processing_timer. It should include a method that will periodicallly get the current cmd_vel from the latest image data
      """
      self.get_logger().info("Processing image data")
      self._current_cmd_vel = processImage(self._current_frame)
        

    def publishCmdVelCb(self):
        """
        This callback will periodically publish the latest cmd_vel data
        """
        self.get_logger().info("Publishing cmd_vel")
        self._cmd_vel_publisher.publish(self._current_cmd_vel)


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