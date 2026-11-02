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
from picar_mission.battery_alarm import alarm as battery_alarm
from picar_mission.battery_routes import register_battery_routes

app = Flask(__name__)
CORS(app, supports_credentials=True)
register_battery_routes(app, battery_alarm, str(mission_bridge.ROOT / 'picar_mission'))
camera = Camera(wait_for_first_frame=False)

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
    response = jsonify(mission_bridge.status())
    response.headers['Cache-Control'] = 'no-store'
    return response


@app.route('/mission/stop', methods=['POST'])
def mission_stop():
    from flask import jsonify
    mission_bridge.cancel('WEB_STOP')
    return jsonify(mission_bridge.status())


@app.route('/dashboard')
def dashboard_page():
    response = send_from_directory(str(mission_bridge.ROOT / 'picar_mission'), 'dashboard.html')
    response.headers['Cache-Control'] = 'no-store'
    return response


@app.route('/dashboard/start', methods=['POST'])
def dashboard_start():
    from flask import jsonify, request
    from urllib.parse import urlsplit
    # A motion-start request must originate from this controller's own page.
    origin = request.headers.get('Origin')
    if origin and urlsplit(origin).netloc != request.host:
        return jsonify(error='이 로봇의 웹 화면에서 시작하세요'), 403
    if not request.is_json:
        return jsonify(error='JSON 요청이 필요합니다'), 415
    body = request.get_json(silent=True)
    if not isinstance(body,dict) or type(body.get('expected_revision')) is not int:
        return jsonify(error='현재 제어 상태를 먼저 확인하세요'), 400
    if body.get('session_id') != mission_bridge.SESSION_ID:
        return jsonify(error='서버가 다시 시작됐습니다. 현재 상태를 확인하고 다시 누르세요'), 409
    try:
        result = mission_bridge.start_mission(body['expected_revision'])
    except (ValueError,RuntimeError) as exc:
        return jsonify(error=str(exc)), 409
    response = jsonify(result)
    response.headers['Cache-Control'] = 'no-store'
    return response


@app.route('/dashboard/assets/<filename>')
def dashboard_asset(filename):
    from flask import abort
    if filename not in ('dashboard.css','dashboard.js'):
        abort(404)
    return send_from_directory(str(mission_bridge.ROOT / 'picar_mission'), filename)


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


from picar_mission.position_routes import register_position_routes
register_position_routes(app, mission_bridge, str(mission_bridge.ROOT / 'picar_mission'))


@app.route('/poses/legacy')
def poses_page():
    return send_from_directory(str(mission_bridge.ROOT / 'picar_mission'), 'poses.html')


@app.route('/poses/data')
def poses_data():
    from flask import jsonify
    import json
    profile = json.loads((mission_bridge.ROOT / 'arm_poses.json').read_text(encoding='utf-8'))
    return jsonify(profile=profile, manual=not mission_bridge.config.auto_approach_enabled,
                   angles=mission_bridge.status().get('commanded_servo_angles_deg'),
                   move=mission_bridge.pose_move_status())


@app.route('/poses/save', methods=['POST'])
def poses_save():
    from flask import jsonify, request
    body = request.get_json(silent=True) or {}
    try:
        saved = mission_bridge.save_pose(body.get('name'))
    except ValueError as exc:
        return jsonify(error=str(exc)), 400
    return jsonify(name=body.get('name'), saved=saved)


@app.route('/poses/move', methods=['POST'])
def poses_move():
    from flask import jsonify, request
    body = request.get_json(silent=True) or {}
    try:
        move = mission_bridge.move_to_pose(body.get('name'))
    except ValueError as exc:
        return jsonify(error=str(exc)), 400
    return jsonify(move=move)


@app.route('/poses/rehearse', methods=['POST'])
def poses_rehearse():
    from flask import jsonify, request
    body = request.get_json(silent=True) or {}
    try:
        move = mission_bridge.rehearse(body.get('direction'))
    except (ValueError, RuntimeError) as exc:
        return jsonify(error=str(exc)), 400
    return jsonify(move=move)


@app.route('/poses/stop', methods=['POST'])
def poses_stop():
    from flask import jsonify
    mission_bridge.stop_pose_move()
    return jsonify(move=mission_bridge.pose_move_status())


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
