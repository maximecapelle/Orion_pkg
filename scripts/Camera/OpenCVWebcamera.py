#!/usr/bin/env python3

import cv2

def main():
    # Create a VideoCapture object to access the webcam
    cap = cv2.VideoCapture(0)

    # Check if the webcam is opened successfully
    if not cap.isOpened():
        print("Failed to open webcam")
        return

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # If the frame was not captured successfully, break the loop
        if not ret:
            print("Failed to capture frame")
            break

        # Display the frame in a window named 'Webcam'
        cv2.imshow('Webcam', frame)

        # Wait for the 'q' key to be pressed to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the VideoCapture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

