import cv2
import numpy as np
from skimage.morphology import skeletonize

im = cv2.imread("src/test_scripts/test.png")

gaussian = cv2.GaussianBlur(im, (5, 5), 0)

hsv = cv2.cvtColor(gaussian, cv2.COLOR_BGR2HSV)

lower_yellow = np.array([28, 80, 30])
upper_yellow = np.array([30, 255, 110])

yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

kernel = np.ones((15, 15), np.uint8)
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
kernel = np.ones((3, 3), np.uint8)
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

skeletonize(yellow_mask)


# contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# for i, contour in enumerate(contours):
#     area = cv2.contourArea(contour)
#     if area > 1000:
       


while True:
  cv2.imshow('Image', im)
  cv2.imshow('Yellow', yellow_mask)
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break