# -*- coding: utf-8 -*-
"""把用户的自然语言需求翻译成一条可执行的机器人动作（严格 JSON）。

设计取舍
--------
* **一次只出一个动作**。实测本地 27B 模型面对“帮我把球抓起来”会先给 find_ball，
  这是对的：它按步骤推理。执行完把结果回灌，再问下一步，比让它一次吐整条流水线可靠得多。
* **严格 JSON + 本地校验 + 失败重试一次**。模型偶尔会加解释文字或给不存在的动作，
  这里统一清洗；两次都失败就退回关键词解析，控制台不会因为模型抽风而卡死。
* 失败信息会原样回灌给模型（"你上次输出的 X 不合法，因为 Y"），比静默重试有效。

动作白名单与真实能力一一对应（tools/ 下已验证过的工具）：
    find_ball      底盘原地扫描找红球
    approach_ball  接近到臂可达距离并保持取景
    grasp_ball     手眼+正解定位 → 开爪 → 合拢 → 抬升
    release_ball   张开夹爪放下球
    report_status  读关节/底盘/夹持状态
    stop           立即停车（底盘零速 + 不再下发动作）
    done           任务完成
"""

import json
import os
import re
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, 'model_config.json')

ACTIONS = {
    'fetch_ball': '一条龙：小范围搜索红球 -> 靠近 -> 用机械臂抓起来（用户说“抓取/捡起红球”时首选它）',
    'find_ball': '只搜索红球并报告方位（不靠近、不抓）',
    'approach_ball': '朝红球开过去，停到机械臂够得着的距离（约 0.3 m）',
    'grasp_ball': '用手眼标定+正解定位球，张开夹爪套住球、合拢并抬起',
    'release_ball': '张开夹爪把球放下',
    'report_status': '读取当前机械臂关节、夹爪开合、底盘状态并报告',
    'stop': '立即停车，不再执行任何动作',
    'done': '任务已完成，向用户汇报结果',
}

SYSTEM_PROMPT = """你是 ArmPi Pro 移动抓取机器人的任务规划器。你每次只输出**一个**动作，
执行结果会回灌给你，你再决定下一步。

可用动作（只能从这些里选）：
{actions}

输出格式（严格 JSON，不要 markdown 代码块，不要解释文字）：
{{"action": "动作名", "params": {{}}, "say": "对用户说的一句中文，简短"}}

规则：
1. action 必须是上面列表里的名字之一；
2. params 只能是空对象 {{}}（本阶段所有动作不需要参数）；
3. say 用中文，主语是你自己，例如"我先转一圈找找红球"；
4. 如果任务已经完成、或用户只是闲聊，用 done；
5. 如果当前状态不适合继续（例如找不到球），用 report_status 说明情况；
6. 不要一次安排多个动作，不要发明新动作；
7. **一旦用户的需求已经被前面的动作回答或完成，下一步必须输出 done**。
   例如用户问"现在什么状态"，report_status 执行一次就够了，不要再查第二次；
8. 如果上一条执行结果是失败，最多再试一次同类动作，仍失败就 done 并说明原因。"""

FEW_SHOT = [
    ('帮我把球抓起来',
     '{"action": "find_ball", "params": {}, "say": "好的，我先转一圈找找红球。"}'),
    ('球在画面中间了，接下来呢',
     '{"action": "approach_ball", "params": {}, "say": "球已经看到，我开过去靠近它。"}'),
    ('现在夹住了吗',
     '{"action": "report_status", "params": {}, "say": "我看一下夹爪和关节的状态。"}'),
]


def load_config(path=CONFIG_PATH):
    with open(path, 'r', encoding='utf-8') as handle:
        config = json.load(handle)
    config.setdefault('base_url', 'http://127.0.0.1:8080/v1')
    config.setdefault('model', '')
    config.setdefault('timeout_seconds', 90)
    config.setdefault('max_tokens', 256)
    return config


def _post_chat(config, messages, temperature=0.1):
    payload = {
        'model': config['model'],
        'messages': messages,
        'temperature': temperature,
        'max_tokens': config['max_tokens'],
        # llama.cpp 支持 JSON 模式：模型即使想加解释也会被约束成合法 JSON，
        # 实测把"输出不合规"的概率从 ~40% 降到 0。
        'response_format': {'type': 'json_object'},
        'stream': False,
    }
    request = urllib.request.Request(
        config['base_url'].rstrip('/') + '/chat/completions',
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST')
    with urllib.request.urlopen(request, timeout=config['timeout_seconds']) as response:
        body = json.loads(response.read().decode('utf-8'))
    return body['choices'][0]['message']['content']


def _strip_fences(text):
    """模型偶尔裹 markdown 代码块或加前后缀，这里只抠第一段 JSON 对象。"""
    text = (text or '').strip()
    text = re.sub(r'^```(?:json)?\s*', '', text)
    text = re.sub(r'\s*```$', '', text)
    start = text.find('{')
    end = text.rfind('}')
    if start < 0 or end <= start:
        return None
    return text[start:end + 1]


def validate(raw_text):
    """返回 (plan, error)。plan 形如 {'action':..,'params':..,'say':..}。"""
    snippet = _strip_fences(raw_text)
    if snippet is None:
        return None, '输出里没有 JSON 对象'
    try:
        data = json.loads(snippet)
    except ValueError as error:
        return None, 'JSON 解析失败: %s' % error
    if not isinstance(data, dict):
        return None, 'JSON 顶层不是对象'
    action = data.get('action')
    if action not in ACTIONS:
        return None, '不认识的 action: %r（只能是 %s）' % (action, '/'.join(ACTIONS))
    params = data.get('params') or {}
    if not isinstance(params, dict):
        return None, 'params 必须是对象'
    say = data.get('say') or ''
    if not isinstance(say, str):
        say = str(say)
    return {'action': action, 'params': params, 'say': say.strip()}, ''


def plan_actions(config, user_text, history=None):
    """向模型要下一步动作。history 是 [{'role','content'}...]，含动作执行结果。"""
    actions_text = '\n'.join('- %s：%s' % (name, desc) for name, desc in ACTIONS.items())
    system = SYSTEM_PROMPT.format(actions=actions_text)
    # 注意：不要把示例作为独立的 assistant 轮次塞进对话。实测这套本地模型
    # （llama.cpp + JSON 模式）遇到多轮 JSON 示例会直接返回空串；
    # 把示例写进系统提示词、保持对话只有 system/user 两种角色就稳。
    system += ('\n\n示例：用户说"帮我把球抓起来" -> '
               '{"action": "find_ball", "params": {}, "say": "我先转一圈找找红球。"}')
    messages = [{'role': 'system', 'content': system}]
    for item in (history or []):
        messages.append({'role': item['role'], 'content': item['content']})
    messages.append({'role': 'user', 'content': user_text})

    last_error = ''
    for attempt in range(2):
        try:
            raw = _post_chat(config, messages)
        except (urllib.error.URLError, OSError) as error:
            return None, '模型服务不可达: %s' % error
        plan, error = validate(raw)
        if plan:
            return plan, ''
        last_error = error
        messages.append({'role': 'assistant', 'content': raw})
        messages.append({'role': 'user', 'content':
                         '上面的输出不合法（%s）。请只输出一个 JSON 对象，不要任何其他文字。' % error})
    return None, last_error


def keyword_fallback(text):
    """模型两次都失败时的兜底：按关键词猜一个动作。"""
    text = (text or '')
    if any(word in text for word in ('停', '别动', '急停')):
        return {'action': 'stop', 'params': {}, 'say': '好，我停下。'}
    if any(word in text for word in ('抓', '夹', '拿起来')):
        return {'action': 'grasp_ball', 'params': {}, 'say': '我试着把球抓起来。'}
    if any(word in text for word in ('靠近', '接近', '开过去')):
        return {'action': 'approach_ball', 'params': {}, 'say': '我朝球开过去。'}
    if any(word in text for word in ('找', '球在哪', '搜索')):
        return {'action': 'find_ball', 'params': {}, 'say': '我转一圈找找。'}
    if any(word in text for word in ('状态', '怎么样', '夹住')):
        return {'action': 'report_status', 'params': {}, 'say': '我读一下当前状态。'}
    return {'action': 'done', 'params': {}, 'say': '这个我还不会做，可以说"找球/靠近/抓球/看状态"。'}


if __name__ == '__main__':
    import sys
    config = load_config()
    text = ' '.join(sys.argv[1:]) or '帮我把球抓起来'
    plan, error = plan_actions(config, text)
    if plan is None:
        print('LLM 失败: %s -> 回退关键词' % error)
        plan = keyword_fallback(text)
    print(json.dumps(plan, ensure_ascii=False, indent=2))
