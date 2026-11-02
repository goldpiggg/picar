"""The same teaching interface is used by the robot and isolated PC preview."""
from urllib.parse import urlsplit
from flask import jsonify, request, send_from_directory


def register_position_routes(app, backend, assets):
    @app.get('/poses')
    def position_page():
        response = send_from_directory(assets, 'positions.html')
        response.headers['Cache-Control'] = 'no-store'
        return response

    @app.get('/positions/assets/<filename>')
    def position_asset(filename):
        from flask import abort
        if filename not in ('positions.css', 'positions.js'):
            abort(404)
        return send_from_directory(assets, filename)

    @app.get('/positions/data')
    def position_data():
        try:
            response = jsonify(backend.position_data())
            response.headers['Cache-Control'] = 'no-store'
            return response
        except (ValueError, OSError) as exc:
            return jsonify(error=str(exc)), 400

    def body():
        origin = request.headers.get('Origin')
        if origin and urlsplit(origin).netloc != request.host:
            raise PermissionError('이 로봇의 저장 화면에서 조작하세요')
        if not request.is_json:
            raise ValueError('JSON 요청이 필요합니다')
        data = request.get_json(silent=True)
        if not isinstance(data, dict):
            raise ValueError('자리와 자세를 확인하세요')
        return data

    def action(callback):
        try:
            return jsonify(callback(body()))
        except PermissionError as exc:
            return jsonify(error=str(exc)), 403
        except (ValueError, OSError, RuntimeError) as exc:
            return jsonify(error=str(exc)), 400

    @app.post('/positions/save')
    def position_save():
        return action(lambda data: dict(saved=backend.save_position(data.get('position'), data.get('pose'))))

    @app.post('/positions/move')
    def position_move():
        return action(lambda data: dict(move=backend.move_to_position(data.get('position'), data.get('pose'),
            data.get('expected_revision'), data.get('session_id'))))

    @app.post('/positions/stop')
    def position_stop():
        return action(lambda data: dict(move=backend.stop_position_move()))
