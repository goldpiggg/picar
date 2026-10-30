#!/usr/bin/env/python
# File name   : app.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date		  : 2025/03/12
from importlib import import_module
import os
from flask import Flask, render_template, Response, send_from_directory
from flask_cors import *

from camera_opencv import Camera
import camera_opencv
import threading
import mission_bridge

app = Flask(__name__)
CORS(app, supports_credentials=True)
camera = Camera()

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/mission/status')
def mission_status():
    from flask import jsonify
    return jsonify(mission_bridge.status())


@app.route('/mission/stop', methods=['POST'])
def mission_stop():
    from flask import jsonify
    mission_bridge.cancel('WEB_STOP')
    return jsonify(mission_bridge.status())


@app.route('/record')
def record_page():
    return send_from_directory(str(mission_bridge.ROOT / 'picar_mission'), 'record.html')


@app.route('/record/status')
def record_status():
    from flask import jsonify
    return jsonify(mission_bridge.recorder.status())


@app.route('/record/<action>', methods=['POST'])
def record_action(action):
    from flask import jsonify, request
    body = request.get_json(silent=True) or {}
    recorder = mission_bridge.recorder
    try:
        if action == 'start':
            result = recorder.start(body.get('side'), body.get('note', ''))
        elif action == 'mark':
            result = recorder.mark(body.get('task'), body.get('label'), body.get('ok', True))
        elif action == 'stop':
            result = recorder.stop(body.get('success'), body.get('stop_distance_cm'), body.get('note'))
        else:
            return jsonify(error='unknown action'), 404
    except ValueError as exc:
        return jsonify(error=str(exc)), 400
    return jsonify(result=result, status=recorder.status())


@app.route('/mission')
def mission_page():
    return send_from_directory(str(mission_bridge.ROOT / 'picar_mission'), 'status.html')

dir_path = os.path.dirname(os.path.realpath(__file__))

@app.route('/api/img/<path:filename>')
def sendimg(filename):
    return send_from_directory(dir_path+'/dist/img', filename)

@app.route('/js/<path:filename>')
def sendjs(filename):
    return send_from_directory(dir_path+'/dist/js', filename)

@app.route('/css/<path:filename>')
def sendcss(filename):
    return send_from_directory(dir_path+'/dist/css', filename)

@app.route('/api/img/icon/<path:filename>')
def sendicon(filename):
    return send_from_directory(dir_path+'/dist/img/icon', filename)

@app.route('/fonts/<path:filename>')
def sendfonts(filename):
    return send_from_directory(dir_path+'/dist/fonts', filename)

@app.route('/<path:filename>')
def sendgen(filename):
    return send_from_directory(dir_path+'/dist', filename)

@app.route('/')
def index():
    return send_from_directory(dir_path+'/dist', 'index.html')

class webapp:
    def __init__(self):
        self.camera = camera

    def modeselect(self, modeInput):
        Camera.modeSelect = modeInput

    def modeselectApp(self, modeInput):
        camera_opencv.APPMode = modeInput

    def colorFindSet(self, H, S, V):
        camera.colorFindSet(H, S, V)

    def colorFindSetApp(self, H, S, V):
        camera.colorFindSetApp(H, S, V)

    def thread(self):
        app.run(host='0.0.0.0', port=5000,threaded=True)

    def startthread(self):
        fps_threading=threading.Thread(target=self.thread)             
        fps_threading.daemon = False
        fps_threading.start()           


if __name__ == "__main__":
    WEB = webapp()
    try:
        WEB.startthread()
    except:
        print("exit")
