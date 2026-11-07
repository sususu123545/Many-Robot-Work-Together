# -*- coding: utf-8 -*-
"""Agent 桥接服务：浏览器 → 本地模型 → 小车（零依赖，标准库实现）。

为什么不复用 server.py 的 FastAPI：
    这台机器上两个 Python 都没装 fastapi/uvicorn，为了跑一个聊天页去装一堆
    依赖不划算，也容易把「控制台起不来」变成新的故障点。这里用 http.server，
    不需要任何第三方包，`python agent_server.py` 就能起。

接口
----
    GET  /                 聊天页面（内嵌 HTML）
    POST /api/chat         {"message": "帮我把球抓起来"} -> 启动一次任务
    GET  /api/mission      当前任务进度（步骤、每步结果）
    POST /api/mission/cancel
    GET  /api/health       隧道/模型/机械臂的可用性快照

模型的闭环在 llm_planner + pi_adapter 里：一次只出一个动作，执行结果回灌，
直到 done/stop 或步数上限。
"""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import llm_planner          # noqa: E402
import pi_adapter           # noqa: E402

MAX_STEPS = 4
STATE_LOCK = threading.Lock()
STATE = {
    "active": False,
    "message": "",
    "status": "idle",
    "steps": [],
    "started": "",
}


def state_snapshot():
    with STATE_LOCK:
        return json.loads(json.dumps(STATE, ensure_ascii=False))


def state_update(**fields):
    with STATE_LOCK:
        STATE.update(fields)


def run_mission(message):
    config = llm_planner.load_config()
    history = []
    steps = []
    state_update(active=True, message=message, status="planning",
                 steps=steps, started=time.strftime("%H:%M:%S"))
    try:
        for index in range(MAX_STEPS):
            plan, error = llm_planner.plan_actions(config, message, history)
            if plan is None:
                plan = llm_planner.keyword_fallback(message)
                plan["say"] = (plan.get("say") or "") + \
                    "（模型输出不合规：%s，已退回关键词）" % (error or '未知')
            action = plan["action"]
            entry = {"action": action, "say": plan.get("say", ""), "result": "",
                     "ok": None, "progress": ""}
            steps.append(entry)
            state_update(status="running", steps=list(steps))
            # 代码层保护：模型有时会重复同一个动作不收敛，连续两次就收尾
            if len(steps) >= 2 and steps[-2]["action"] == action and action != "done":
                entry["result"] = "（连续两次同一动作，判定已完成，停止）"
                entry["ok"] = True
                state_update(status="done", steps=list(steps))
                return
            if action in ("done", "stop"):
                entry["result"] = "结束"
                entry["ok"] = True
                state_update(status="done", steps=list(steps))
                return
            outcome = pi_adapter.run(
                action, plan.get("params") or {},
                progress=lambda note, target=entry: target.__setitem__("progress", note))
            entry["result"] = outcome.get("summary", "")
            entry["ok"] = bool(outcome.get("ok"))
            state_update(steps=list(steps))
            history.append({"role": "assistant",
                            "content": json.dumps(plan, ensure_ascii=False)})
            history.append({"role": "user",
                            "content": "动作 %s 的执行结果：%s。如果这个结果已经回答了用户的需求，"
                                       "下一步只能输出 done。" % (action, entry["result"])})
            if not outcome.get("ok"):
                state_update(status="failed", steps=list(steps))
                return
        state_update(status="done", steps=list(steps))
    except Exception as error:                     # noqa: BLE001
        with STATE_LOCK:
            STATE["steps"] = STATE["steps"] + [
                {"action": "error", "say": "", "result": str(error), "ok": False}]
            STATE["status"] = "failed"
    finally:
        state_update(active=False)


PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>ArmPi Pro · 模型指挥台</title>
<style>
 body{margin:0;background:#111418;color:#e6e6e6;font:15px/1.6 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
 .wrap{max-width:860px;margin:0 auto;padding:24px}
 h1{font-size:18px;font-weight:600;margin:0 0 4px}
 .sub{color:#8b95a3;font-size:13px;margin-bottom:18px}
 .box{background:#1a1f26;border:1px solid #262d36;border-radius:10px;padding:14px;margin-bottom:14px}
 textarea{width:100%;box-sizing:border-box;background:#0f1317;color:#e6e6e6;border:1px solid #2a323c;
   border-radius:8px;padding:10px;font:14px/1.5 inherit;resize:vertical;min-height:64px}
 button{background:#2f6fed;color:#fff;border:0;border-radius:8px;padding:9px 18px;font-size:14px;
   cursor:pointer;margin-top:8px}
 button:disabled{background:#39424e;cursor:default}
 button.ghost{background:#2a323c}
 .step{border-left:3px solid #2f6fed;padding:8px 12px;margin:8px 0;background:#151a20;border-radius:0 8px 8px 0}
 .step.fail{border-color:#e05252}.step.ok{border-color:#2fbf71}
 .act{font-family:ui-monospace,Consolas,monospace;color:#7fb3ff;font-size:13px}
 .say{margin:2px 0}.res{color:#a9b4c2;font-size:13px;margin-top:4px}
 .prog{color:#6f7a88;font-size:12px}
 .pill{display:inline-block;padding:2px 8px;border-radius:999px;background:#232a33;color:#9aa5b1;font-size:12px}
</style></head><body><div class="wrap">
<h1>ArmPi Pro · 模型指挥台</h1>
<div class="sub">说一句需求，本地模型（Ternary-Bonsai-2-27B）自己规划动作并下发给小车。</div>
<div class="box">
 <textarea id="msg" placeholder="例如：帮我把球抓起来 / 现在什么状态 / 先找球再靠近"></textarea>
 <button id="send">发送给模型</button>
 <button id="cancel" class="ghost">取消任务</button>
 <span id="stat" class="pill">空闲</span>
</div>
<div id="steps"></div>
</div>
<script>
const $ = id => document.getElementById(id);
async function post(url, body){
  const r = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                              body: JSON.stringify(body||{})});
  return r.json();
}
$('send').onclick = async () => {
  const text = $('msg').value.trim();
  if(!text) return;
  const res = await post('/api/chat', {message:text});
  if(!res.ok) alert(res.error || '发送失败');
};
$('cancel').onclick = () => post('/api/mission/cancel');
async function tick(){
  try{
    const s = await (await fetch('/api/mission')).json();
    $('stat').textContent = ({idle:'空闲',planning:'模型规划中',running:'执行中',
                              done:'已完成',failed:'失败',cancelled:'已取消'})[s.status] || s.status;
    $('send').disabled = s.active;
    const html = (s.steps||[]).map(st => {
      const cls = st.ok === true ? 'ok' : (st.ok === false ? 'fail' : '');
      return `<div class="step ${cls}">
        <div class="act">${st.action}</div>
        ${st.say ? `<div class="say">${st.say}</div>` : ''}
        ${st.result ? `<div class="res">${st.result}</div>` : ''}
        ${st.progress && !st.result ? `<div class="prog">${st.progress}</div>` : ''}
      </div>`;
    }).join('');
    $('steps').innerHTML = html;
  }catch(e){}
}
setInterval(tick, 1200); tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    server_version = 'ArmPiAgent/1.0'

    def log_message(self, fmt, *args):               # 安静一点
        pass

    def _send(self, code, payload, content_type='application/json'):
        body = payload if isinstance(payload, bytes) else \
            json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', content_type + '; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            return self._send(200, PAGE.encode('utf-8'), 'text/html')
        if self.path == '/api/mission':
            return self._send(200, state_snapshot())
        if self.path == '/api/health':
            return self._send(200, health())
        return self._send(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length).decode('utf-8') if length else '{}'
        try:
            payload = json.loads(raw or '{}')
        except ValueError:
            return self._send(400, {'ok': False, 'error': 'JSON 不合法'})

        if self.path == '/api/chat':
            message = (payload.get('message') or '').strip()
            if not message:
                return self._send(400, {'ok': False, 'error': '消息为空'})
            with STATE_LOCK:
                if STATE['active']:
                    return self._send(409, {'ok': False, 'error': '上一个任务还在跑'})
            threading.Thread(target=run_mission, args=(message,), daemon=True).start()
            return self._send(200, {'ok': True, 'message': message})

        if self.path == '/api/mission/cancel':
            state_update(status='cancelled')
            return self._send(200, {'ok': True})

        return self._send(404, {'ok': False, 'error': 'not found'})


def health():
    """快速自检：模型服务、rosbridge 隧道、相机隧道。"""
    import socket
    import urllib.request
    result = {}
    try:
        with urllib.request.urlopen(
                llm_planner.load_config()['base_url'].rstrip('/') + '/models', timeout=5) as r:
            result['model'] = 'ok' if r.status == 200 else 'http %d' % r.status
    except Exception as error:                     # noqa: BLE001
        result['model'] = 'unreachable: %s' % error
    for name, port in (('rosbridge', 19091), ('camera', 19099)):
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=3):
                result[name] = 'ok'
        except OSError as error:
            result[name] = 'down: %s' % error
    return result


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8020
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print('模型指挥台已启动: http://127.0.0.1:%d' % port, flush=True)
    print('自检: %s' % json.dumps(health(), ensure_ascii=False), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
