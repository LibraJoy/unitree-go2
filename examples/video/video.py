import cv2
import numpy as np

height, width = 720, 1280  # You can adjust the size
img = np.zeros((height, width, 3), dtype=np.uint8)
cv2.imshow('video', img)
#print(f"Shape: {img.shape}, Dimensions: {img.ndim}, Type: {img.dtype}, Size: {img.size}")
cv2.waitKey(1)

from display_video_channel import display

display()
