from flask import Flask, Response, render_template, make_response, request
import threading
import time
import cv2
import numpy as np
from multiprocessing import Process, Queue
from visionRetrieve import Display
from flask_cors import CORS
import socket

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

@app.route('/post_orientation', methods=['POST'])
def post_orientation():
    global latest_orientation
    data = request.get_json()

    if data and all(k in data for k in ('x', 'y', 'z', 'w')):
        latest_orientation = data
        print(f"[VR] Orientation received: {latest_orientation}")
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
