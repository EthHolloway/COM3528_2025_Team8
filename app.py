from flask import Flask, Response, render_template, make_response
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
        print("Access the server locally via 127.0.0.1:5000 for testing, access over wifi via http://<localhost>:5000")
        app.run(host='0.0.0.0', port=5000)
    finally:
        vision_proc.terminate()
        vision_proc.join()
