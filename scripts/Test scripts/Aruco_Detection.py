
####
# Import statements
####

import cv2
import numpy as np
from Aruco_Definitions import arucoDict, cameraMatrix, distCoeffs
from collections import deque
from scipy.spatial.transform import Rotation as R
import time
import matplotlib.pyplot as plt

####
# Perform Pose Estimation of Aruco's
####

# Define the marker length
marker_length = 0.05

# Create video capture object
cap = cv2.VideoCapture(0)
# Get the frame rate of the video
frame_rate = cap.get(cv2.CAP_PROP_FPS)
# Get time step between images
delta_t = 1/frame_rate

# Initialise pose estimation lists
euler_angs = deque(maxlen=2)
positions = deque(maxlen=2)

# Initialise state plot
# fig = plt.figure()
# ax = fig.add_subplot()
fig, axs = plt.subplots()

# Initialize counter
counter = 0.0

# Capture video
while True:
    
    # Read the current frame from the video capture
    ret, frame = cap.read()
    
    # Break loop if no frames are captured
    if not ret:
        break
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect Aruco markers and ChArUco corners
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict)
    
    # If markers are detected
    if len(corners) > 0.0:
        
        # Draw detected markers
        frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        
        # Estimate marker pose
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_length, cameraMatrix, distCoeffs)

        # Draw the axis for each marker
        for i in range(len(rvecs)):
            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], marker_length/3)
            
            # Obtain Positions
            positions.append(tvecs[0][0])
            # Obtain Rodrigues Parameters
            rodri_param = rvecs[0][0]
            # Transform to rotation matrix
            rot_mat, _ = cv2.Rodrigues(rodri_param)
            r = R.from_matrix( rot_mat )
            euler_angs.append(r.as_euler('zyx', degrees=False))
            
            # Perform State Estimate
            # Check if we have enough values to perform differencing
            if len(positions) == 2:
                
                # Increase counter
                counter = counter + 1
                
                # Obtain position
                pos = positions[-1]
                # Obtain velocity
                velo = (positions[-1] - positions[-2])/delta_t
                # Obtain attitude
                att = euler_angs[-1]
                # Obtain angular velocity
                omega = (euler_angs[-1] - euler_angs[-2])/delta_t
                
                ####
                # Print position and attitude on the image
                ####
                
                # Print settings
                fontFace = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.5
                color = (255, 0, 0)  # Blue color
                thickness = 2
                lineType = cv2.LINE_AA
                
                # Print position
                org = (50, 50)
                text_pos = "ID: {} | Position: ({:.2f}, {:.2f}, {:.2f} [m])".format(ids[i][0], pos[0], pos[1], pos[2])
                cv2.putText(frame, text_pos, org, fontFace, fontScale, color, thickness, lineType)
                # Print velocity
                org = (50, 70)
                text_velo = "ID: {} | Velocity: ({:.2f}, {:.2f}, {:.2f} [m/s])".format(ids[i][0], velo[0], velo[1], velo[2])
                cv2.putText(frame, text_velo, org, fontFace, fontScale, color, thickness, lineType)
                # Print attitude
                org = (50, 90)
                text_pos = "ID: {} | Attitude XYZ: ({:.2f}, {:.2f}, {:.2f} [deg])".format(ids[i][0], np.rad2deg(att[0]), np.rad2deg(att[1]), np.rad2deg(att[2]) )
                cv2.putText(frame, text_pos, org, fontFace, fontScale, color, thickness, lineType)
                # Print angular velocity
                org = (50, 110)
                text_velo = "ID: {} | Angular Rate: ({:.2f}, {:.2f}, {:.2f} [deg/s])".format(ids[i][0], np.rad2deg(velo[0]), np.rad2deg(velo[1]), np.rad2deg(velo[2]) )
                cv2.putText(frame, text_velo, org, fontFace, fontScale, color, thickness, lineType)
                
                # Plot the figure
                axs.scatter(counter, np.round(np.linalg.norm(pos), 2), color='r')
                fig.canvas.draw()
                axs.set_xlim(left=max(0, counter - 25), right = counter + 5)
                
                time.sleep(delta_t)

    
    # Create a named window with resizable flag
    cv2.namedWindow('Aruco Detection', cv2.WINDOW_NORMAL)

    # Display the image with marker axes
    cv2.imshow('Aruco Detection', frame)
    
    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# Release the video capture object and close any open windows
cap.release()
cv2.destroyAllWindows()

# show plot
plt.show()