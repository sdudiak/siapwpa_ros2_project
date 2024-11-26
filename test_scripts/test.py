import cv2
import numpy as np
import matplotlib.pyplot as plt

# img = cv2.imread("src/test_scripts/test.png")
img = cv2.imread("src/test_scripts/test1.png")

# src = np.float32([
#     [665, 550], 
#     [1160, 550],
#     [0, 755], 
#     [1920, 755]
#     ])

# dst = np.float32([
#     [0, 1080],
#     [1920, 1080],
#     [0, 0],
#     [1920, 0]
#     ])

src = np.float32([
    [520, 560], 
    [1300, 560],
    [0, 840], 
    [1920, 840]
    ])

dst = np.float32([
    [0, 0],
    [1920, 0],
    [0, 1080],
    [1920, 1080]
    ])
    
img_size = (img.shape[1], img.shape[0])

matrix = cv2.getPerspectiveTransform(src, dst)

birdseye = cv2.warpPerspective(img, matrix, img_size)

birdseye_rgb = cv2.cvtColor(birdseye, cv2.COLOR_BGR2RGB)

hsv = cv2.cvtColor(birdseye_rgb, cv2.COLOR_BGR2HSV)
print(type(hsv))
lower_yellow = np.array([88, 120, 41])
upper_yellow = np.array([95, 255, 55])
binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

print(np.where(binary_mask != 0))

kernel = np.ones((11, 11), np.uint8)
binary_mask = cv2.dilate(binary_mask, kernel, iterations=2)

# plt.imshow(hsv)
plt.imshow(binary_mask, cmap='gray')
# plt.imshow(img)
plt.axis('off')
plt.show()