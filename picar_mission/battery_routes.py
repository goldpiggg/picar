"""Read battery status and change only its warning-sound preference."""
from urllib.parse import urlsplit
from flask import jsonify, request, send_from_directory


def register_battery_routes(app, controller, assets, preview=False):
    def status():
        result = controller.snapshot()
        result['preview'] = preview
        return result

    @app.get('/battery/status')
    def battery_status():
        response = jsonify(status())
        response.headers['Cache-Control'] = 'no-store'
        return response

    @app.post('/battery/sound')
    def battery_sound():
        origin = request.headers.get('Origin')
        if origin and urlsplit(origin).netloc != request.host:
            return jsonify(error='이 로봇의 화면에서 설정하세요'), 403
        body = request.get_json(silent=True)
        if not request.is_json or not isinstance(body, dict) or type(body.get('enabled')) is not bool:
            return jsonify(error='경고음 켜기 또는 끄기를 선택하세요'), 400
        try:
            controller.set_sound_enabled(body['enabled'])
            return jsonify(status())
        except (ValueError, OSError, RuntimeError) as exc:
            return jsonify(error=str(exc)), 400

    @app.get('/battery/assets/<filename>')
    def battery_asset(filename):
        from flask import abort
        if filename not in ('battery_alarm.css', 'battery_alarm.js'):
            abort(404)
        return send_from_directory(assets, filename)
