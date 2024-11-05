import cv2
import numpy as np

def process_image(img_path):
    # Load the image
    img = cv2.imread(img_path)
    height, width, _ = img.shape

    # Define the region of interest
    rect_height = 180
    rect_top = height // 2 - rect_height // 2 + 90  # Shift ROI further down
    rect_bottom = rect_top + rect_height

    # Crop and blur the ROI
    rect_img = img[rect_top:rect_bottom, 0:width]
    rect_img = cv2.GaussianBlur(rect_img, (5, 5), 0)

    # Convert to HSV color space
    hsv = cv2.cvtColor(rect_img, cv2.COLOR_BGR2HSV)
    
    # Define the yellow color range
    lower_yellow = np.array([25, 50, 50])
    upper_yellow = np.array([255, 255, 255])
    binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Dilate mask to increase line thickness
    kernel = np.ones((5, 5), np.uint8)
    binary_mask = cv2.dilate(binary_mask, kernel, iterations=1)

    # Clean up mask
    binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
    binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

    # Detect lines using Hough Transform
    lines = cv2.HoughLinesP(binary_mask, 1, np.pi / 180, threshold=20, minLineLength=10, maxLineGap=10)
    line_frame = img.copy()

    # Draw the ROI rectangle
    cv2.rectangle(line_frame, (0, rect_top), (width, rect_bottom), (255, 0, 0), 2)

    if lines is not None:
        left_x = float('inf')
        right_x = float('-inf')
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                left_x = min(left_x, x1, x2)
                right_x = max(right_x, x1, x2)

        if left_x < float('inf') and right_x > float('-inf'):
            # Calculate the midpoint
            cx_full = int((left_x + right_x) // 2)
            cy_full = (rect_top + rect_bottom) // 2

            # Draw the centroid and highlight detected lines
            cv2.circle(line_frame, (cx_full, cy_full), 5, (0, 255, 0), -1)
            line_frame[rect_top:rect_bottom, :][binary_mask > 0] = (0, 255, 255)

    # Display the images
    cv2.imshow("Original Image", img)
    cv2.imshow("Binary Mask", binary_mask)
    cv2.imshow("Detected Lines with Centroids", line_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Path to the image file
process_image("/home/developer/ros2_ws/src/images/frame01.png")
