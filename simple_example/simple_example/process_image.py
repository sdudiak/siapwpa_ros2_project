import cv2
import numpy as np
from geometry_msgs.msg import Twist
def processImage(img:np.ndarray) -> Twist:
  """
  This method returns the appropriate velocity data based on the incoming image frame
  """
  # TODO: image processing here
  output_cmd_vel = Twist()
  output_cmd_vel.angular.x = 0.
  output_cmd_vel.angular.y = 0.
  output_cmd_vel.angular.z = 0.
  output_cmd_vel.linear.x = 1. # Just for testing
  output_cmd_vel.linear.y = 0.
  output_cmd_vel.linear.z = 0.
  return output_cmd_vel