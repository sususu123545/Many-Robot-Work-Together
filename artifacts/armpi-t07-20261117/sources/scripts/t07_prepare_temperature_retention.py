"""Retain only still-valid REAL temperature observations, never a new timestamp."""
import json,hashlib,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];previous=ROOT/'logs/t07/runs/20261117T044329Z_32e1672d';run=ROOT/'logs/t07/runs/20261117T050800Z_f8ef182f';patch=run/'driver_patch';patch.mkdir(exist_ok=False)
for name in ['arm_node.py','gripper_diag.py','base.launch.py','servo_protocol.py']:shutil.copy2(previous/'driver_patch'/name,patch/name)
path=patch/'arm_node.py';source=path.read_text(encoding='utf-8')
old='''        temperature = self.board.temperature(ident)
        temperature_time = time.monotonic()'''
new='''        retained = False
        temperature_read_error = None
        try:
            temperature = self.board.temperature(ident)
            temperature_time = time.monotonic()
        except TimeoutError as error:
            # A missed temperature reply need not discard a still-valid physical
            # observation. Never retain an encoder or refresh the old timestamp.
            with self.state_lock:
                previous = dict(self.health.samples.get(name, {}))
            now = time.monotonic()
            try:
                usable = (0 <= previous['temperature'] < self.temp_max
                          and 0 <= now - previous['temperature_time'] < 1.5)
            except (KeyError, TypeError, ValueError):
                usable = False
            if not usable:
                raise
            temperature = previous['temperature']
            temperature_time = previous['temperature_time']
            retained = True
            temperature_read_error = str(error)'''
assert source.count(old)==1;source=source.replace(old,new)
old="'position_time': position_time, 'temperature_time': temperature_time}"
new="'position_time': position_time, 'temperature_time': temperature_time,\n                'temperature_retained': retained,\n                'temperature_read_error': temperature_read_error}"
assert source.count(old)==1;source=source.replace(old,new);compile(source,'arm_node_temperature_retention','exec');path.write_bytes(source.encode())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
proposal={'operator_approval_for_restart':'approved_existing_session_readonly_restart; continued authorized control repair, no startup motion/torque','latch_stop_before_restart':True,'baseline_arm_source_sha256':sha(previous/'driver_patch/arm_node.py'),'baseline_launch_sha256':sha(patch/'base.launch.py'),'baseline_helper_source_sha256':sha(patch/'gripper_diag.py'),'baseline_protocol_sha256':sha(patch/'servo_protocol.py'),'protocol_patch':True,'module_filename':'gripper_diag.py','scope':'Temperature TimeoutError may retain previous measured value ONLY with original timestamp and age<1.5s; encoder faults and explicit errors still stop; original whole-cycle validity and 65C limit unchanged; startup gates OFF','physical_training_export':False}
(patch/'proposal.json').write_text(json.dumps(proposal,indent=2)+'\n',encoding='utf-8');print(json.dumps(proposal))
