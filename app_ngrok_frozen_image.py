#!/usr/bin/env python3
import time
import cv2
import numpy as np
from multiprocessing import Process, Queue
from flask import Flask, Response, render_template, stream_with_context
from flask_cors import CORS
from visionRetrieve import Display

app = Flask(__name__)
CORS(app)

# a queue to hold the latest stereo frame
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
    return render_template('index_ngrok_frozen_image.html')

@app.route('/video_feed')
def video_feed():
    """stream MJPEG from the frame_queue as multipart/x-mixed-replace."""
    def generate():
        while True:
            frame = frame_queue.get()              
            success, jpeg = cv2.imencode('.jpg', frame)
            if not success:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')
    return Response(
        stream_with_context(generate()),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    # start the vision subprocess
    vision_proc = Process(target=vision_process, args=(frame_queue,))
    vision_proc.start()
    try:
        print("Serving Flask on http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000)
    finally:
        vision_proc.terminate()
        vision_proc.join()