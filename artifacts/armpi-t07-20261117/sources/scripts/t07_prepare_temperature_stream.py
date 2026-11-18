import json,hashlib,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];previous=ROOT/'logs/t07/runs/20261117T050800Z_f8ef182f/driver_patch';run=ROOT/'logs/t07/runs/20261117T064820Z_39f8e694';patch=run/'driver_patch';patch.mkdir(exist_ok=False)
for name in ['arm_node.py','base.launch.py','servo_protocol.py','gripper_diag.py']:shutil.copy2(previous/name,patch/name)
p=patch/'arm_node.py';source=p.read_text(encoding='utf-8');old_hash=hashlib.sha256(p.read_bytes()).hexdigest()
needle="        self.status_pub = self.create_publisher(String, '/arm_controller/status', 10)";assert source.count(needle)==1
source=source.replace(needle,needle+"\n        self.temperature_pub = self.create_publisher(String, '/arm_controller/temperature_feedback', 20)")
needle="        return {'raw': raw, 'temperature': temperature,";assert source.count(needle)==1
source=source.replace(needle,"""        if not retained:
            # Emit this physical read now, before other joint reads delay the
            # complete-cycle status. Do not change any health/admission state.
            self.temperature_pub.publish(String(data=json.dumps({
                'source':'physical_temperature_read','joint':name,'servo_id':ident,
                'temperature_C':temperature,'temperature_time':temperature_time,
                'retained':False})))
"""+needle)
compile(source,'arm_node_temperature_stream','exec');p.write_bytes(source.encode())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
proposal={'operator_approval_for_restart':'approved_existing_session_readonly_restart; continue authorized feedback repair, all startup gates OFF','latch_stop_before_restart':True,'module_filename':'gripper_diag.py','baseline_arm_source_sha256':old_hash,'baseline_launch_sha256':sha(patch/'base.launch.py'),'baseline_helper_source_sha256':sha(patch/'gripper_diag.py'),'scope':'Publish each new actual temperature with original timestamp immediately; no added sensor query or motion; health samples, complete-cycle command admission, age limits and65C stop unchanged','restore_stage':False}
(patch/'proposal.json').write_text(json.dumps(proposal,indent=2)+'\n',encoding='utf-8');print(json.dumps(proposal))
