from flask import Flask, Response, render_template
import threading
import time
import cv2
import numpy as np
from visionRetrieve import display  # Replace with your actual script name

app = Flask(__name__)

# Initialize ROS camera node
print("[INFO] Initializing ROS video stream...")
robot_display = display()
print("[INFO] ROS video node started.")

# Shared stereo frame buffer
stereo_frame = None

# Lock to avoid race conditions
frame_lock = threading.Lock()


def update_frames():
    global stereo_frame
    print("[THREAD] Frame update thread started.")

    while not robot_display.stop:
        left = robot_display.left_cam()
        right = robot_display.right_cam()

        if left is not None and right is not None:
            if isinstance(left, np.ndarray) and isinstance(right, np.ndarray):
                with frame_lock:
                    stereo_frame = np.hstack((left, right))
                    print("[DEBUG] Stereo frame updated")
            else:
                print("[WARN] Left or right frame is not a valid image")
        else:
            print("[WARN] Waiting for both left and right frames...")

        time.sleep(0.033)  # ~30 FPS

    print("[THREAD] Frame update thread exiting.")


@app.route('/')
def index():
    print("[REQUEST] Client accessed '/' - Serving WebXR page.")
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    print("[REQUEST] Client accessed '/video_feed' - Starting video stream.")
    
    def generate():
        global stereo_frame
        while True:
            with frame_lock:
                if stereo_frame is not None:
                    success, jpeg = cv2.imencode('.jpg', stereo_frame)
                    if not success:
                        print("[ERROR] JPEG encoding failed.")
                        continue
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                else:
                    print("[INFO] Stereo frame not ready yet.")
            time.sleep(0.033)

    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    try:
        print("[MAIN] Starting frame update thread...")
        t = threading.Thread(target=update_frames)
        t.daemon = True
        t.start()

        print("[MAIN] Starting Flask server on http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C received. Shutting down...")

    # finally:
        # robot_display.stop = True
        # print("[CLEANUP] Waiting for background thread to exit...")
        # t.join()
        # print("[CLEANUP] Shutdown complete.")
