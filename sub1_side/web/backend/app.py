"""
ROBO CHEF — Flask Backend Server
포트 5000에서 실행. Firebase + ROS2 브릿지 역할.
"""
import threading
import os
import sys
import subprocess

from flask import Flask, jsonify, request, abort
from flask_cors import CORS

import firebase_client as fb
import robot_bridge as rb
import config
import ros2_coord_client as coord_client


app = Flask(__name__)
CORS(app)


# ── Firebase & 로봇 초기화 ─────────────────────────────────────────────────────

def startup():
    fb.init()
    fb.push_log("INFO", "Flask 백엔드 서버 시작", source="system")
    fb.update_robot_state({
        "robot_status":   "IDLE",
        "gripper_status": "OPEN",
        "speed_scale":    30,
        "current_recipe": None,
        "current_step":   0,
        "error_code":     "NONE",
        "mode":           "STANDBY",
        "joint_positions": [0, 0, 90, 0, 90, 0],
        "tcp_position":   {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0},
    })
    rb.start_state_monitor()
    coord_client.start()   # 영속 ROS2 서비스 클라이언트 시작
    # 주문 처리는 메인노드 firebase_bridge 가 RTDB /orders(pending) 감시 후 전담


# ─────────────────────────────────────────────
#  REST API
# ─────────────────────────────────────────────

# ── /api/recipes ─────────────────────────────

@app.route("/api/recipes", methods=["GET"])
def list_recipes():
    return jsonify(fb.get_all_recipes())


@app.route("/api/recipes/<recipe_id>", methods=["GET"])
def get_recipe(recipe_id):
    r = fb.get_recipe(recipe_id)
    if r is None:
        abort(404, "Recipe not found")
    return jsonify(r)


@app.route("/api/recipes", methods=["POST"])
def create_recipe():
    data = request.get_json(force=True)
    recipe_id = data.get("recipe_id")
    if not recipe_id:
        abort(400, "recipe_id required")
    fb.upsert_recipe(recipe_id, data)
    return jsonify({"ok": True, "recipe_id": recipe_id}), 201


# ── /api/orders ──────────────────────────────

@app.route("/api/orders", methods=["GET"])
def list_orders():
    limit = int(request.args.get("limit", 50))
    return jsonify(fb.get_orders(limit))


@app.route("/api/orders/<order_id>", methods=["GET"])
def get_order(order_id):
    o = fb.get_order(order_id)
    if o is None:
        abort(404, "Order not found")
    return jsonify(o)


@app.route("/api/orders", methods=["POST"])
def create_order():
    data = request.get_json(force=True)
    recipe_id = data.get("recipe_id")
    items     = data.get("items", [])
    total     = data.get("total", 0)
    if not recipe_id:
        abort(400, "recipe_id required")
    order_id = fb.create_order(recipe_id, items, total)
    fb.push_log("INFO", f"새 주문 접수: {order_id}", source="kiosk")
    # 조리 트리거는 메인노드 firebase_bridge 가 RTDB /orders 감시로 전담(웹 발행 없음)

    return jsonify({"ok": True, "order_id": order_id}), 201


@app.route("/api/orders/<order_id>/status", methods=["PATCH"])
def update_order_status(order_id):
    data   = request.get_json(force=True)
    status = data.get("status")
    if not status:
        abort(400, "status required")
    fb.update_order_status(order_id, status)
    return jsonify({"ok": True})


# ── /api/robot ───────────────────────────────

@app.route("/api/robot/state", methods=["GET"])
def robot_state():
    return jsonify(fb.get_robot_state())


@app.route("/api/robot/coords", methods=["GET"])
def get_robot_coords():
    """
    영속 ROS2 클라이언트로 /robo_chef/get_coords 서비스 호출.
    subprocess 방식 대신 process 내 persistent node를 사용해
    DDS participant 반복 생성/소멸 문제를 방지한다.
    반환: { "joint":[j1..j6], "tcp":[x,y,z,rx,ry,rz], "timestamp":"...", "source":"ros2" }
    """
    data = coord_client.get_coords(timeout=5.0)
    if "error" in data:
        return jsonify(data), 503
    data["source"] = "ros2"
    return jsonify(data)


@app.route("/api/robot/stop", methods=["POST"])
def robot_stop():
    ok = rb.stop_robot()
    fb.update_robot_state({"robot_status": "IDLE", "current_step": 0, "current_recipe": None})
    return jsonify({"ok": ok})


@app.route("/api/robot/home", methods=["POST"])
def robot_home():
    threading.Thread(
        target=lambda: rb.move_joint([0, 0, 90, 0, 90, 0]),
        daemon=True,
    ).start()
    fb.push_log("MOVE", "홈 포지션으로 이동", source="system")
    return jsonify({"ok": True})


@app.route("/api/robot/mode/autonomous", methods=["POST"])
def set_autonomous():
    ok = rb.set_robot_mode_autonomous()
    fb.update_robot_state({"mode": "AUTONOMOUS" if ok else "MANUAL"})
    return jsonify({"ok": ok})


# ── /api/logs ────────────────────────────────

@app.route("/api/logs", methods=["GET"])
def get_logs():
    limit = int(request.args.get("limit", 50))
    return jsonify(fb.get_recent_logs(limit))


@app.route("/api/logs", methods=["POST"])
def add_log():
    """ROS2 노드 등 외부에서 로그를 전송할 엔드포인트"""
    data = request.get_json(force=True)
    fb.push_log(
        data.get("level", "INFO"),
        data.get("message", ""),
        data.get("source", "external"),
    )
    return jsonify({"ok": True}), 201


# ── Health check ─────────────────────────────

@app.route("/api/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "service": "robo-chef-backend"})


# ── ROS2 상태 조회 ──

@app.route("/api/ros2/status", methods=["GET"])
def ros2_status():
    """ROS2 토픽 설정 및 상태 반환."""
    import subprocess, os
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(config.ROS2_DOMAIN_ID)
    # 토픽 목록 확인 (타임아웃 2초)
    try:
        r = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=3, env=env
        )
        topics = r.stdout.strip().split("\n") if r.returncode == 0 else []
    except Exception:
        topics = []

    return jsonify({
        "order_topic":   config.ROS2_ORDER_TOPIC,
        "status_topic":  config.ROS2_STATUS_TOPIC,
        "domain_id":     config.ROS2_DOMAIN_ID,
        "msg_format":    config.ORDER_MSG_FORMAT,
        "active_topics": topics,
    })


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

if __name__ == "__main__":
    startup()
    app.run(
        host=config.FLASK_HOST,
        port=config.FLASK_PORT,
        debug=config.FLASK_DEBUG,
        use_reloader=False,
    )
