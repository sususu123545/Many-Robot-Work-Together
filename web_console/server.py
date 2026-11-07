# -*- coding: utf-8 -*-
"""
多车协作 · 送水控制台 —— 后端服务（阶段 1：单机 + 模拟执行器）

架构说明（阶段 1 务实版）:
    浏览器 ──HTTP/WS──> 本服务 ──> 任务队列 ──> RobotAdapter 执行
                                    ├── SimAdapter : 本机模拟机器人（现在用，无小车也能看全流程）
                                    └── PiAdapter  : 树莓派车端对接（阶段 1 部署时实现，接口已预留）

运行（在隔离 venv 中）:
    python -m uvicorn server:app --host 0.0.0.0 --port 8010
"""
import asyncio
import json
import queue
import threading
import time
import uuid
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel

import llm_planner
import pi_adapter

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

# ---------------------------------------------------------------------------
# 可配置区：座位表 / 物品表（阶段 2 将引入人脸识别自动定位，替换"座位=位置"的简化）
# ---------------------------------------------------------------------------
SEATS = {
    "user":  "主控台（你的位置）",
    "seat1": "1 号座位",
    "seat2": "2 号座位",
    "seat3": "3 号座位",
}
OBJECTS = {
    "water": "一瓶水",
}

# ---------------------------------------------------------------------------
# 指令解析：中文自然语言 -> 结构化任务（关键词匹配；阶段 2 可换 LLM 解析）
# ---------------------------------------------------------------------------
CN_NUM = {"1": "1", "一": "1", "2": "2", "二": "2", "两": "2", "3": "3", "三": "3"}


def parse_command(text: str) -> dict:
    """解析 '给我拿水' / '帮 3 号座位拿杯水' 之类的指令。"""
    text = (text or "").strip()
    if not text:
        raise ValueError("指令为空")

    obj = None
    for key, label in OBJECTS.items():
        if key in text or any(w in text for w in {"water": ["水"]}[key]):
            obj = key
            break
    if obj is None:
        raise ValueError(f"暂时只认识这些物品：{list(OBJECTS.values())}")

    target = "user"
    for n in ("3", "2", "1"):  # 先匹配两位数避免 '1' 吃掉 '13'
        if (n + "号" in text) or (n + " 号" in text) or ("座位" + n in text):
            target = "seat" + n
            break
    else:
        cn = [CN_NUM[c] for c in text if c in CN_NUM]
        if cn:
            target = "seat" + cn[-1]
    if "我" in text and target == "user":
        target = "user"

    return {"object": obj, "target": target}


# ---------------------------------------------------------------------------
# 任务存储与机器人状态（内存版；重启即清空，阶段 1 足够）
# ---------------------------------------------------------------------------
TASK_STATUSES = [
    "queued",        # 排队中
    "moving_to_pick",  # 前往取水点
    "grasping",      # 抓取中
    "moving_to_seat",  # 配送中
    "placing",       # 放置物品
    "done",          # 完成
    "failed",        # 失败
    "cancelled",     # 已取消
]
STATUS_CN = {
    "queued": "排队中", "moving_to_pick": "前往取水点", "grasping": "抓取中",
    "moving_to_seat": "配送中", "placing": "放置物品", "done": "已完成",
    "failed": "失败", "cancelled": "已取消",
}

tasks: dict = {}            # task_id -> task dict
tasks_order: list = []      # 保持展示顺序
cancel_flags: set = set()   # 需要取消的 task_id
robot_state = {
    "name": "ArmpiPro-01（模拟）",
    "battery": 87,
    "position": "充电点",
    "current_task": None,
    "mode": "simulator",
}
_lock = threading.Lock()


def _snapshot() -> dict:
    with _lock:
        return {
            "robot": dict(robot_state),
            "tasks": [dict(tasks[tid]) for tid in tasks_order],
            "mission": _mission_snapshot(),
        }


def _set_task(task_id: str, **fields):
    with _lock:
        tasks[task_id].update(fields)


# ---------------------------------------------------------------------------
# RobotAdapter：执行层抽象。SimAdapter 现用；PiAdapter 阶段 1 部署时实现。
# ---------------------------------------------------------------------------
class SimAdapter:
    """模拟机器人：按真实状态机节奏推进，方便本机验收全流程交互。"""

    STEP_CN = {
        "moving_to_pick": "正在前往取水点…",
        "grasping": "机械臂正在抓取水瓶…",
        "moving_to_seat": "正在送往 {}…",
        "placing": "正在放下水瓶…",
    }

    def execute(self, task: dict, cancel_check) -> str:
        target_label = SEATS.get(task["target"], task["target"])
        for status, seconds in (("moving_to_pick", 4), ("grasping", 3),
                                ("moving_to_seat", 5), ("placing", 2)):
            if cancel_check():
                return "cancelled"
            _set_task(task["id"], status=status,
                      note=self.STEP_CN[status].format(target_label))
            if status == "moving_to_pick":
                with _lock:
                    robot_state["position"] = "水桌"
            elif status == "moving_to_seat":
                with _lock:
                    robot_state["position"] = target_label
            for _ in range(seconds * 2):
                if cancel_check():
                    return "cancelled"
                time.sleep(0.5)
        return "done"


class PiAdapter:
    """树莓派车端对接（预留）。部署阶段 1 时实现：把任务下发给车端 ROS 节点，
    轮询 /robot_status 反馈进度。写代码时保持与 SimAdapter 相同的 execute() 签名。"""

    def execute(self, task: dict, cancel_check) -> str:
        raise NotImplementedError("车端未连通：先解决树莓派供电/SSH，再实现此适配器")


ADAPTER = SimAdapter()


def _worker():
    """任务队列消费线程。"""
    while True:
        task_id = _q.get()
        task = tasks.get(task_id)
        if not task or task["status"] != "queued":
            continue
        if task_id in cancel_flags:
            _set_task(task_id, status="cancelled", note="已取消")
            cancel_flags.discard(task_id)
            continue
        with _lock:
            robot_state["current_task"] = task_id
        _set_task(task_id, status="moving_to_pick", note="开始执行")
        try:
            result = ADAPTER.execute(task, lambda: task_id in cancel_flags)
        except Exception as e:  # noqa: BLE001
            _set_task(task_id, status="failed", note=f"执行异常: {e}")
            result = "failed"
        if task_id in cancel_flags:
            result = "cancelled"
            cancel_flags.discard(task_id)
        status = "done" if result == "done" else result
        notes = {"done": "已送达 " + SEATS.get(task["target"], ""),
                 "cancelled": "任务已取消"}
        _set_task(task_id, status=status, note=notes.get(status, task.get("note", "")))
        with _lock:
            robot_state["current_task"] = None
            robot_state["position"] = "充电点"


_q: queue.Queue = queue.Queue()
threading.Thread(target=_worker, daemon=True).start()


# ---------------------------------------------------------------------------
# FastAPI 应用
# ---------------------------------------------------------------------------
app = FastAPI(title="多车协作 · 送水控制台")


class TaskIn(BaseModel):
    command: str


# ---------------------------------------------------------------------------
# 阶段 2：自然语言 -> 本地模型 -> 真车动作（模型自己发任务）
# ---------------------------------------------------------------------------
MAX_STEPS = 6            # 一次需求最多让模型推进几步，防止它绕圈

mission_state = {
    "active": False,
    "message": "",
    "status": "idle",     # idle / planning / running / done / failed
    "steps": [],          # [{action, say, result}]
    "step_index": 0,
}
_mission_lock = threading.Lock()


def _mission_update(**fields):
    with _mission_lock:
        mission_state.update(fields)


def _mission_snapshot():
    with _mission_lock:
        return json.loads(json.dumps(mission_state, ensure_ascii=False))


def _run_mission(message: str):
    """模型闭环：规划一个动作 -> 执行 -> 把结果回灌 -> 再规划。"""
    config = llm_planner.load_config()
    history = []
    steps = []
    _mission_update(active=True, message=message, status="planning",
                    steps=steps, step_index=0)
    try:
        for step in range(MAX_STEPS):
            plan, error = llm_planner.plan_actions(config, message, history)
            if plan is None:
                plan = llm_planner.keyword_fallback(message)
                plan["say"] = (plan.get("say") or "") +                     "（模型输出不合规，已退回关键词解析）"
                error = error
            action = plan["action"]
            entry = {"action": action, "say": plan.get("say", ""), "result": ""}
            steps.append(entry)
            _mission_update(status="running", step_index=step,
                            steps=list(steps))
            if action in ("done", "stop"):
                entry["result"] = "结束"
                _mission_update(status="done", steps=list(steps))
                return
            outcome = pi_adapter.run(action, plan.get("params") or {},
                                     progress=lambda note: entry.__setitem__("progress", note))
            entry["result"] = outcome.get("summary", "")
            entry["ok"] = bool(outcome.get("ok"))
            _mission_update(steps=list(steps))
            history.append({"role": "assistant",
                            "content": json.dumps(plan, ensure_ascii=False)})
            history.append({"role": "user",
                            "content": "动作 %s 的执行结果：%s" % (action, entry["result"])})
            if not outcome.get("ok"):
                _mission_update(status="failed", steps=list(steps))
                return
        _mission_update(status="done", steps=list(steps))
    except Exception as error:  # noqa: BLE001
        _mission_update(status="failed",
                        steps=list(steps) + [{"action": "error", "say": "",
                                              "result": str(error)}])
    finally:
        _mission_update(active=False)


class ChatIn(BaseModel):
    message: str


@app.post("/api/chat")
def chat(body: ChatIn):
    text = (body.message or "").strip()
    if not text:
        return {"ok": False, "error": "消息为空"}
    with _mission_lock:
        if mission_state["active"]:
            return {"ok": False, "error": "上一个任务还在执行，先等它结束或取消"}
    threading.Thread(target=_run_mission, args=(text,), daemon=True).start()
    return {"ok": True, "message": text}


@app.get("/api/mission")
def mission():
    return _mission_snapshot()


@app.post("/api/mission/cancel")
def mission_cancel():
    _mission_update(status="cancelled")
    return {"ok": True}


@app.post("/api/task")
def submit_task(body: TaskIn):
    try:
        parsed = parse_command(body.command)
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    task_id = uuid.uuid4().hex[:8]
    task = {
        "id": task_id,
        "command": body.command.strip(),
        "object": parsed["object"],
        "object_label": OBJECTS[parsed["object"]],
        "target": parsed["target"],
        "target_label": SEATS[parsed["target"]],
        "status": "queued",
        "note": "排队中",
        "created": time.strftime("%H:%M:%S"),
    }
    with _lock:
        tasks[task_id] = task
        tasks_order.insert(0, task_id)
    _q.put(task_id)
    return {"ok": True, "task": task}


@app.post("/api/cancel/{task_id}")
def cancel_task(task_id: str):
    if task_id not in tasks:
        return {"ok": False, "error": "任务不存在"}
    if tasks[task_id]["status"] in ("done", "failed", "cancelled"):
        return {"ok": False, "error": "该任务已结束"}
    cancel_flags.add(task_id)
    return {"ok": True}


@app.get("/api/status")
def status():
    return _snapshot()


@app.get("/api/config")
def config():
    return {"seats": SEATS, "objects": OBJECTS,
            "status_cn": STATUS_CN, "mode": robot_state["mode"]}


# --- WebSocket：每 0.8s 推一次全局快照（线程写状态，这里只读，简单可靠） ---
_clients: set = set()


async def _broadcaster():
    while True:
        if _clients:
            snap = json.dumps(_snapshot(), ensure_ascii=False)
            dead = []
            for ws in list(_clients):
                try:
                    await ws.send_text(snap)
                except Exception:  # noqa: BLE001
                    dead.append(ws)
            for ws in dead:
                _clients.discard(ws)
        await asyncio.sleep(0.8)


@app.websocket("/ws/status")
async def ws_status(ws: WebSocket):
    await ws.accept()
    _clients.add(ws)
    try:
        await ws.send_text(json.dumps(_snapshot(), ensure_ascii=False))
        while True:
            await ws.receive_text()  # 保活；前端不发送业务消息
    except WebSocketDisconnect:
        pass
    finally:
        _clients.discard(ws)


@app.on_event("startup")
async def _start():
    asyncio.create_task(_broadcaster())


@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8010)
