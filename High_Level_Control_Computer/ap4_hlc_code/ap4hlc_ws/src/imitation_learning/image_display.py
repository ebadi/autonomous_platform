import cv2
import pandas as pd
import time
import numpy as np
import sys
 
image_df = pd.read_pickle(sys.argv[1])

cv2.namedWindow("Images", cv2.WINDOW_NORMAL)
cv2.namedWindow("depth", cv2.WINDOW_NORMAL)

fps = 10
interval = 1 / fps
orb = cv2.ORB_create(nfeatures=3000, scaleFactor=1.2, nlevels=8, edgeThreshold=15)
colormap = cv2.COLORMAP_JET

# Iterate through each row and display the images
for index, row in image_df.iterrows():
    start_time = time.time()
    timestamp = row["Timestamp"]
    image_data = row["Image"]
    depth = row["Depth"]
    steering_angle = row["Steering_angle"]
    imu = row["IMU"]

    # image_data = 255 -image_data
    # image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    height, width, _ = image_data.shape

    kp = orb.detect(image_data, None)
    for point in kp:
        point.pt = (point.pt[0], point.pt[1])
        point.size *= 0.01

    image_data = cv2.drawKeypoints(
        image_data,
        kp,
        None,
        color=(0, 255, 0),
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )

    # image_data = cv2.drawKeypoints(image_data, kp, None, color=(0,255,0),flags=0)
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)

    depth_colormap = cv2.applyColorMap(depth_norm.astype(np.uint8), colormap)

    # Overlay the steering angle on the image
    # cv2.putText(image_data, f"Steering Angle: {steering_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the image with the steering angle
    cv2.imshow("Images", image_data)
    cv2.imshow("depth", depth_colormap)

    # Wait for a key press; press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    # Calculate the time elapsed during the iteration
    elapsed_time = time.time() - start_time

    # Pause to achieve the desired frame rate
    time.sleep(max(0, interval - elapsed_time))
# Close all OpenCV windows
cv2.destroyAllWindows()
