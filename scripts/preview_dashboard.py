#!/usr/bin/env python3
"""PC-only UI preview with simulated data. Never imports robot hardware."""
from pathlib import Path
import time
import uuid
import sys
from threading import RLock
from flask import Flask, jsonify, request, send_from_directory

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from picar_mission.battery_alarm import BatteryAlarm
from picar_mission.battery_routes import register_battery_routes
app = Flask(__name__)


class PreviewBatteryAlarm(BatteryAlarm):
    def snapshot(self):
        data = super().snapshot()
        data['sounding'] = data['low_voltage'] and data['sound_enabled']
        data['sample_age_seconds'] = 0.
        return data


preview_battery = PreviewBatteryAlarm(ROOT.parent/'tmp'/'battery-preview'/'sound.json')
preview_battery.update_voltage(7.4)
register_battery_routes(app,preview_battery,str(ROOT/'picar_mission'),preview=True)


@app.post('/battery/preview/voltage')
def preview_voltage():
    from urllib.parse import urlsplit
    origin = request.headers.get('Origin')
    if origin and urlsplit(origin).netloc != request.host:
        return jsonify(error='미리보기 화면에서 설정하세요'),403
    body = request.get_json(silent=True)
    volts = body.get('volts') if isinstance(body,dict) else None
    if type(volts) not in (int,float) or volts not in (5.8,7.4):
        return jsonify(error='가상 전압을 선택하세요'),400
    preview_battery.update_voltage(volts)
    return jsonify(**preview_battery.snapshot(),preview=True)

lock = RLock()
started = None
cancelled_at = None
attempt = 0
revision = 0
SESSION_ID = uuid.uuid4().hex
mode = 'READY'


def status():
    with lock:
        age = 0. if started is None else (cancelled_at or time.monotonic())-started
        state = mode
        direction, distance, transfers = None,80.,0
        if started is not None:
            if age < 2: state = 'WAIT_QR'
            elif age < 8:
                state, direction, distance = 'DRIVING','LEFT',max(31.,80.-(age-2)*8.2)
            elif age < 10: state,direction,distance = 'PULSE_SETTLE','LEFT',30.3
            elif age < 14: state,direction,distance = 'ARM_TRANSFER','LEFT',30.3
            elif age < 18: state,direction,distance,transfers = 'ARM_TRANSFER','LEFT',30.3,1
            else: state,direction,distance,transfers = 'COMPLETE','LEFT',30.3,2
        if mode=='CANCELLED': state = mode
        finished = state in ('COMPLETE','CANCELLED')
        if state=='COMPLETE': age = 18.
        return dict(state=state,first_direction=direction,error=None,
            target_stop_latched=started is not None and age>=10,
            arm={'completed_transfers':transfers},
            dashboard=dict(state=state,attempt=attempt,started=started is not None,finished=finished,
                can_start=started is None or finished,elapsed_seconds=age,
                session_id=SESSION_ID,control_revision=revision,distance_cm=round(distance,1),remaining_cm=0. if age>=10 else round(max(0.,distance-30.),1),
                target_distance_cm=30.,distance_source='FINAL' if age>=10 else 'LIVE',
                camera_online=True,arm_enabled=True,arm_calibrated=True))


@app.get('/dashboard')
@app.get('/')
def page():
    text = (ROOT/'picar_mission/dashboard.html').read_text(encoding='utf-8')
    return text.replace('<main>','<main><div style="background:#e6b751;color:#3d3217;text-align:center;padding:7px;font-size:12px">PC 미리보기 · 가상 데이터 · 실제 로봇에 연결되지 않음</div>')


@app.get('/dashboard/assets/<filename>')
def asset(filename):
    return send_from_directory(ROOT/'picar_mission',filename)


@app.get('/mission/status')
def data(): return jsonify(status())


@app.post('/dashboard/start')
def start():
    global started, cancelled_at, attempt, mode, revision
    with lock:
        body = request.get_json(silent=True) or {}
        if body.get('expected_revision') != revision or body.get('session_id') != SESSION_ID:
            return jsonify(error='현재 상태를 확인하고 다시 시작하세요'),409
        if status()['dashboard']['can_start']:
            started, cancelled_at, mode = time.monotonic(),None,'RUNNING'
            attempt += 1
            revision += 1
        return jsonify(status())


@app.post('/mission/stop')
def stop():
    global mode, cancelled_at, revision
    with lock:
        mode, cancelled_at = 'CANCELLED',time.monotonic()
        revision += 1
        return jsonify(status())


@app.get('/video_feed')
def camera():
    # A clearly labeled static preview image, not a fabricated live camera feed.
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 480">
    <defs><pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse"><path d="M40 0H0V40" fill="none" stroke="#365347" stroke-width=".5"/></pattern></defs>
    <rect width="640" height="480" fill="#203b31"/><rect width="640" height="480" fill="url(#grid)"/>
    <path d="M195 440L270 160M445 440L370 160" stroke="#b7c6a3" stroke-width="3"/>
    <path d="M225 345H415" stroke="#d9b364" stroke-width="7"/>
    <rect x="266" y="90" width="108" height="90" rx="4" fill="#dce6ce"/>
    <path d="M285 115h22v22h-22zM333 115h22v22h-22zM285 145h22v22h-22zM326 146h12v12h-12zM344 157h11v11h-11z" fill="#214535"/>
    <text x="320" y="232" text-anchor="middle" fill="#dce6ce" font-family="sans-serif" font-size="17">LEFT</text>
    <text x="320" y="270" text-anchor="middle" fill="#8eac97" font-family="sans-serif" font-size="12">SIMULATED CAMERA / UI PREVIEW</text></svg>'''
    return svg,200,{'Content-Type':'image/svg+xml','Cache-Control':'no-store'}


if __name__ == '__main__':
    print('PC preview only: http://127.0.0.1:5081/dashboard',flush=True)
    app.run(host='127.0.0.1',port=5081,threaded=True)
