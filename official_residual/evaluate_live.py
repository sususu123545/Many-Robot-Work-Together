"""Re-evaluate a saved residual checkpoint with the official native controller."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time
import torch
from stable_baselines3 import PPO
from live_env import OfficialResidualEnv,ROOT
from train_live import evaluate,gateway

p=argparse.ArgumentParser();p.add_argument('checkpoint',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
torch.set_num_threads(1)
control_lock=open(ROOT/'.official-control.lock','a')
try:fcntl.flock(control_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Another residual training/evaluation process owns control')
a.out.parent.mkdir(parents=True,exist_ok=True)
pidfile=Path('/mnt/llm/matrix_install/official-input.pid')
if pidfile.exists():
    pid=int(pidfile.read_text())
    try:
        command=Path(f'/proc/{pid}/cmdline').read_bytes()
        if b'official-zsl1-control.py' in command and b'daemon' in command:
            subprocess.run(['kill','-TERM',str(pid)],check=True);time.sleep(.15)
    except FileNotFoundError:pass
env=None
try:
    model=PPO.load(a.checkpoint,device='cpu')
    env=OfficialResidualEnv(log_path=a.out.with_suffix('.jsonl'))
    result=evaluate(env,model,seeds=(201,202));result['checkpoint']=str(a.checkpoint.resolve())
    a.out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    if env:env.close()
    gateway('neutral')
    subprocess.Popen(['/usr/bin/python3','/mnt/llm/matrix_install/official-zsl1-control.py','daemon'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
