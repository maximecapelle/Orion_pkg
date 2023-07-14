import cv2
from flask import Flask, Response

print("OpenCV version:", cv2.__version__)
app = Flask(__name__)

def generate_frames():
    # Define the video source
    video_source = 0  # Replace with the appropriate video source (e.g., file path or camera index)

    # Initialize the VideoCapture object
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Yield the frame to be sent as a response
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()

@app.route('/')
def stream_video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8765)
