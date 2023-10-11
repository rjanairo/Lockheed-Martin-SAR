import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

# Create video capture object for webcam
cap = cv2.VideoCapture(0)  # 0 is default camera

if not cap.isOpened():
        raise Exception("Could not open video capture device (camera).")
frameCount = 0
redetectInterval = 10

# Shi-Tomasi Parameters
# Max corners = negative number if you don't want a max
shitomasi_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=7, blockSize=7)

# Read the first frame
ret, frame = cap.read()

# Create an empty mask to draw on later
mask = np.zeros_like(frame)

# Get the frame dimensions
frame_height, frame_width = frame.shape[:2]

# Define the height of the rectangular ROI (covers the entire height)
roi_height = frame_height

# Define the width of the rectangular ROI (you can adjust this value)
roi_width = 400

# Calculate the x-coordinates to define the rectangular ROI
roi_x1 = (frame_width - roi_width) // 2  # Center the ROI horizontally
roi_x2 = roi_x1 + roi_width

# Define the Lucas-Kanade window size 
lk_params = dict(winSize=(200, 200), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Adjust the first frame
initGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
initROI = initGray[0:roi_height, roi_x1:roi_x2]
edges = cv2.goodFeaturesToTrack(initROI, mask=None, **shitomasi_params)

while True:
    # Read the frame from the video
    ret, frame = cap.read()

    # Break the loop if there are no more frames to read
    if not ret:
        break

    frameCount += 1
    # Perform feature re-detection every 'redetectInterval' frames
    if frameCount % redetectInterval == 0:
        initGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        initROI = initGray[0:roi_height, roi_x1:roi_x2]
        edges = cv2.goodFeaturesToTrack(initROI, mask=None, **shitomasi_params)

    # Blur frame to reduce background noise
    blur = cv2.blur(frame, ksize=(5, 5))

    # Draw the ROI rectangle on the frame
    cv2.rectangle(frame, (roi_x1, 0), (roi_x2, roi_height), (0, 255, 0), 2)
    # Split camera in two
    cv2.line(mask, (frame_width // 2, 0), (frame_width // 2, frame_height), (255, 0, 0), 2)

    # Median value of the entire frame
    medianVal = np.median(blur)
    # Lower threshold to either 0 or 70% of median value, whichever is greater
    lower = int(max(0, 0.7 * medianVal))
    # Experiment with these values such as adding +50 to upper
    upper = int(min(255, 1.3 * medianVal))

    # Grayscale image before edge detection
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    roi = gray[0:roi_height, roi_x1:roi_x2]
    # Calculate Lucas-Kanade optical flow
    new_edges, status, _ = cv2.calcOpticalFlowPyrLK(initROI, roi, edges, None, **lk_params)

    # Select good points
    good_new = new_edges[status == 1]
    good_old = edges[status == 1]

    # Draw optical flow lines on the frame
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 0, 255), 2)
        frame = cv2.circle(frame, (int(a), int(b)), 9, (0, 0, 255), -1)

    # Overlay the optical flow lines on the frame
    output = cv2.add(frame, mask)

    # Show the frame with optical flow visualization
    cv2.imshow('Optical Flow', output)

    # Update the previous frame and corners for the next iteration
    initGray = gray.copy()
    edges = good_new.reshape(-1, 1, 2)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
