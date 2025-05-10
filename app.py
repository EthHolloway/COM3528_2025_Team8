from flask import Flask, Response, render_template
import threading
import time
import cv2
import numpy as np
from multiprocessing import Process, Queue
from visionRetrieve import Display
import socket

app = Flask(__name__)

# Use queues to share frames between processes
frame_queue = Queue(maxsize=1)


def vision_process(queue):
    robot_display = Display()
    print("[INFO] Vision node started in subprocess.")

    while not robot_display.stop:
        left = robot_display.left_cam()
        right = robot_display.right_cam()

        # print(left)
        # print(right)

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
    def generate():
        while True:
            if not frame_queue.empty():
                frame = frame_queue.get()
                success, jpeg = cv2.imencode('.jpg', frame)
                if not success:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.033)

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    vision_proc = Process(target=vision_process, args=(frame_queue,))
    vision_proc.start()

    try:
        print("Access the server from other devices at: http://" + socket.gethostbyname(socket.gethostname()) + ":5000")
        app.run(host='0.0.0.0', port=5000)
    finally:
        vision_proc.terminate()
        vision_proc.join()
