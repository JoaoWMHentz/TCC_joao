"""
api.py — rotas HTTP da interface de operação.

Sessão      GET  /api/status            estado geral
            GET  /api/maps              mapas salvos
            DELETE /api/maps/<name>     exclui um mapa salvo (e seus destinos/rotas)
            POST /api/session/mapping   {name, explore?, strategy?}  cria um mapa novo
                                        (explore=true: já explorando sozinho)
            POST /api/session/navigation {name} abre um mapa salvo
            POST /api/session/stop
            GET  /api/logs/<processo>   últimas linhas de base | slam | nav | explore
Exploração  POST /api/explore/start     {strategy?} exploração automática (só no mapeamento)
                                        strategy: wall_bfs (padrão) | pocket | frontier
            POST /api/explore/stop
            GET  /api/explore/log       decisões do explorador nesta sessão
Mapa        GET  /api/map               PNG (base64) + metadados
Controle    POST /api/goal              {x, y, theta}
            POST /api/cancel
            POST /api/initial_pose      {x, y, theta}
            POST /api/teleop            {linear, angular}
Destinos    GET/POST /api/points        DELETE /api/points/<id>   POST /api/points/<id>/go
Rotas       GET/POST /api/routes        DELETE /api/routes/<id>   POST /api/routes/<id>/run
Histórico   GET  /api/history           GET /api/history/summary
"""

from __future__ import annotations

import math

import db
from flask import Blueprint, jsonify, request
from launcher import DEFAULT_STRATEGY, MAP_NAME_RE
import psycopg2
import ros_bridge

api = Blueprint('api', __name__, url_prefix='/api')
launcher = None  # injetado pelo app


class ApiError(Exception):
    def __init__(self, message: str, status: int = 400):
        super().__init__(message)
        self.status = status


@api.errorhandler(ApiError)
def _api_error(exc: ApiError):
    return jsonify({'error': str(exc)}), exc.status


@api.errorhandler(psycopg2.Error)
def _db_error(exc):
    return jsonify({'error': f'Banco de dados indisponível: {exc.__class__.__name__}'}), 503


def _body() -> dict:
    return request.get_json(silent=True) or {}


def _number(body: dict, key: str, default=None) -> float:
    value = body.get(key, default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ApiError(f'"{key}" deve ser numérico') from None
    if not math.isfinite(number):
        raise ApiError(f'"{key}" deve ser finito')
    return number


def _node():
    node = ros_bridge.node()
    if node is None:
        raise ApiError('ROS não inicializado', 503)
    return node


def _active_map() -> str:
    if launcher.mode != 'navigation' or launcher.map_name is None:
        raise ApiError('Abra um mapa em modo navegação primeiro', 409)
    return launcher.map_name


# ── Sessão ───────────────────────────────────────────────────────────────────

def status_payload() -> dict:
    node = ros_bridge.node()
    return {
        **launcher.status(),
        'ros_ok': node is not None,
        'mission': node.mission_status() if node else None,
        'hardware': node.hardware() if node else {},
    }


@api.get('/status')
def get_status():
    return jsonify(status_payload())


@api.get('/maps')
def get_maps():
    return jsonify(launcher.list_maps())


@api.delete('/maps/<name>')
def delete_map(name: str):
    if not MAP_NAME_RE.match(name):
        raise ApiError('Nome de mapa inválido')
    if launcher.map_name == name and launcher.mode != 'idle':
        raise ApiError('Este mapa está em uso; pare a sessão antes de excluí-lo', 409)
    if not launcher.map_path(name).exists():
        raise ApiError(f'Mapa "{name}" não encontrado', 404)
    db.delete_map_data(name)        # antes do arquivo: se o banco falhar, nada some
    ok, message = launcher.delete_map(name)
    if not ok:
        raise ApiError(message, 409)
    return jsonify({'ok': True})


@api.post('/session/<mode>')
def start_session(mode: str):
    if mode == 'stop':
        node = ros_bridge.node()
        if node:
            node.cancel()
            node.map_name = None
        launcher.stop_session()
        return jsonify({'ok': True})
    body = _body()
    ok, message = launcher.start_session(mode, (body.get('name') or '').strip(),
                                         explore=bool(body.get('explore')),
                                         strategy=_strategy(body))
    if not ok:
        raise ApiError(message, 409)
    node = ros_bridge.node()
    if node:
        node.clear_map()
        node.map_name = launcher.map_name
    return jsonify({'ok': True})


@api.get('/logs/<name>')
def get_logs(name: str):
    return jsonify(launcher.logs(name))


# ── Exploração automática ────────────────────────────────────────────────────

@api.post('/explore/<action>')
def explore(action: str):
    if action == 'stop':
        launcher.stop_explore()
        return jsonify({'ok': True})
    if action != 'start':
        raise ApiError('ação inválida', 404)
    node = ros_bridge.node()
    if node:
        node.cancel()            # o explorador passa a mandar os destinos
    ok, message = launcher.start_explore(_strategy(_body()))
    if not ok:
        raise ApiError(message, 409)
    return jsonify({'ok': True})


@api.get('/explore/log')
def explore_log():
    return jsonify(launcher.explore_log())


def _strategy(body: dict) -> str:
    return (body.get('strategy') or DEFAULT_STRATEGY).strip()


def _take_control():
    """Encerra a exploração automática diante de um comando manual.

    Comando manual (destino, rota, teleoperação, cancelar) encerra a
    exploração automática, para os dois não disputarem o Nav2.
    """
    if launcher.running('explore'):
        launcher.stop_explore()


# ── Mapa ─────────────────────────────────────────────────────────────────────

@api.get('/map')
def get_map():
    data = _node().get_map()
    if data is None:
        raise ApiError('Mapa ainda não disponível', 404)
    return jsonify(data)


# ── Controle ─────────────────────────────────────────────────────────────────

@api.post('/goal')
def send_goal():
    body = _body()
    _take_control()
    target = {'name': 'destino marcado', 'x': _number(body, 'x'), 'y': _number(body, 'y'),
              'theta': _number(body, 'theta', 0.0)}
    ok, message = _node().start_mission('goal', 'Destino no mapa', [target])
    if not ok:
        raise ApiError(message, 503)
    return jsonify({'ok': True})


@api.post('/cancel')
def cancel():
    _take_control()
    _node().cancel()
    return jsonify({'ok': True})


@api.post('/initial_pose')
def initial_pose():
    body = _body()
    _node().set_initial_pose(_number(body, 'x'), _number(body, 'y'), _number(body, 'theta', 0.0))
    return jsonify({'ok': True})


@api.post('/teleop')
def teleop():
    body = _body()
    linear = max(-0.8, min(0.8, _number(body, 'linear', 0.0)))
    angular = max(-1.0, min(1.0, _number(body, 'angular', 0.0)))
    if linear or angular:
        _take_control()
    _node().teleop(linear, angular)
    return jsonify({'ok': True})


# ── Destinos ─────────────────────────────────────────────────────────────────

@api.get('/points')
def list_points():
    if launcher.map_name is None:
        return jsonify([])
    return jsonify(db.list_points(launcher.map_name))


@api.post('/points')
def create_point():
    map_name = _active_map()
    body = _body()
    name = (body.get('name') or '').strip()
    if not name:
        raise ApiError('Informe um nome para o destino')
    if body.get('here'):
        pose = _node().get_pose()
        if not pose['available']:
            raise ApiError('Posição do robô indisponível (ele está localizado?)', 409)
        x, y, theta = pose['x'], pose['y'], pose['theta']
    else:
        x, y, theta = _number(body, 'x'), _number(body, 'y'), _number(body, 'theta', 0.0)
    try:
        point_id = db.create_point(map_name, name, x, y, theta)
    except psycopg2.errors.UniqueViolation:
        raise ApiError(f'Já existe um destino chamado "{name}"', 409) from None
    return jsonify({'ok': True, 'id': point_id}), 201


@api.delete('/points/<int:point_id>')
def delete_point(point_id: int):
    if not db.delete_point(point_id):
        raise ApiError('Destino não encontrado', 404)
    return jsonify({'ok': True})


@api.post('/points/<int:point_id>/go')
def go_to_point(point_id: int):
    point = db.get_point(point_id)
    if point is None:
        raise ApiError('Destino não encontrado', 404)
    if point['map'] != _active_map():
        raise ApiError('Este destino pertence a outro mapa', 409)
    _take_control()
    ok, message = _node().start_mission('point', point['name'], [dict(point)])
    if not ok:
        raise ApiError(message, 503)
    return jsonify({'ok': True})


# ── Rotas ────────────────────────────────────────────────────────────────────

@api.get('/routes')
def list_routes():
    if launcher.map_name is None:
        return jsonify([])
    return jsonify(db.list_routes(launcher.map_name))


@api.post('/routes')
def create_route():
    map_name = _active_map()
    body = _body()
    name = (body.get('name') or '').strip()
    point_ids = body.get('point_ids') or []
    if not name:
        raise ApiError('Informe um nome para a rota')
    if len(point_ids) < 2 or not all(isinstance(i, int) for i in point_ids):
        raise ApiError('Uma rota precisa de pelo menos dois destinos')
    try:
        route_id = db.create_route(map_name, name, point_ids)
    except psycopg2.errors.UniqueViolation:
        raise ApiError(f'Já existe uma rota chamada "{name}"', 409) from None
    return jsonify({'ok': True, 'id': route_id}), 201


@api.delete('/routes/<int:route_id>')
def delete_route(route_id: int):
    if not db.delete_route(route_id):
        raise ApiError('Rota não encontrada', 404)
    return jsonify({'ok': True})


@api.post('/routes/<int:route_id>/run')
def run_route(route_id: int):
    route = db.get_route(route_id)
    if route is None:
        raise ApiError('Rota não encontrada', 404)
    if route['map'] != _active_map():
        raise ApiError('Esta rota pertence a outro mapa', 409)
    points = [dict(p) for p in db.route_points(route)]
    if len(points) < 2:
        raise ApiError('A rota tem menos de dois destinos válidos', 409)
    _take_control()
    ok, message = _node().start_mission('route', route['name'], points)
    if not ok:
        raise ApiError(message, 503)
    return jsonify({'ok': True})


# ── Histórico ────────────────────────────────────────────────────────────────

@api.get('/history')
def history():
    return jsonify(db.list_history(int(request.args.get('limit', 50))))


@api.get('/history/summary')
def history_summary():
    return jsonify(db.history_summary())
