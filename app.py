from flask import Flask, Response, render_template, make_response, request
import threading
import time
import cv2
import numpy as np
from multiprocessing import Process, Queue
from visionRetrieve import Display
from flask_cors import CORS
import socket
import math

# Attempt ROS import
try:
    import rospy
    from std_msgs.msg import Float64
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] rospy not available, skipping ROS integration.")

app = Flask(__name__)
CORS(app)

frame_queue = Queue(maxsize=1)

def vision_process(queue):
    robot_display = Display()
    print("[INFO] Vision node started in subprocess.")

    while not robot_display.stop:
        left = robot_display.left_cam()
        right = robot_display.right_cam()

        if left is not None and right is not None:
            stereo_frame = np.hstack((left, right))
            if not queue.full():
                queue.put(stereo_frame)
        time.sleep(0.033)

    print("[INFO] Vision process shutting down.")

latest_orientation = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'w': 1.0
}

# Initialize ROS if available
if ROS_AVAILABLE:
    try:
        rospy.init_node('webxr_miro_bridge', anonymous=True)
        yaw_pub = rospy.Publisher('/miro/command/head_yaw', Float64, queue_size=10)
        pitch_pub = rospy.Publisher('/miro/command/head_pitch', Float64, queue_size=10)
        print("[ROS] Publishers ready.")
    except Exception as e:
        ROS_AVAILABLE = False
        print(f"[ERROR] Could not initialize ROS: {e}")

def quaternion_to_yaw_pitch(x, y, z, w):
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    return yaw, pitch

@app.route('/post_orientation', methods=['POST'])
def post_orientation():
    global latest_orientation
    data = request.get_json()

    if data and all(k in data for k in ('x', 'y', 'z', 'w')):
        latest_orientation = data
        x, y, z, w = data['x'], data['y'], data['z'], data['w']
        yaw, pitch = quaternion_to_yaw_pitch(x, y, z, w)

        print(f"[VR] Orientation received: yaw={yaw:.2f}, pitch={pitch:.2f}")

        if ROS_AVAILABLE:
            try:
                yaw_pub.publish(Float64(yaw))
                pitch_pub.publish(Float64(pitch))
            except Exception as e:
                print(f"[ERROR] Failed to publish to ROS: {e}")
        else:
            print("[ROS] Skipped: ROS not available or not initialized")

        return {'status': 'ok'}, 200
    else:
        return {'error': 'Invalid data'}, 400

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    if not frame_queue.empty():
        frame = frame_queue.get()
        success, jpeg = cv2.imencode('.jpg', frame)
        if success:
            response = make_response(jpeg.tobytes())
            response.headers['Content-Type'] = 'image/jpeg'
            response.headers['Access-Control-Allow-Origin'] = '*'
            return response
    return '', 204  # No content if no frame

if __name__ == '__main__':
    vision_proc = Process(target=vision_process, args=(frame_queue,))
    vision_proc.start()

    try:
        print("Access the server from other devices at: https://<your-ngrok-subdomain>.ngrok.io")
        app.run(host='0.0.0.0', port=5000)
    finally:
        vision_proc.terminate()
        vision_proc.join()
