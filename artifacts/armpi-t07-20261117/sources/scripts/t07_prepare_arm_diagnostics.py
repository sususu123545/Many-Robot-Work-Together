"""Prepare stopped-only read diagnostics and persistent last control-TX evidence."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'logs/t07/runs/20261117T044329Z_32e1672d';patch=run/'driver_patch';patch.mkdir(exist_ok=False)
helper='''"""Read through the existing bus owner only, under latched stop and OFF gates."""
import json,time
from .servo_protocol import JOINTS
def read_diagnostics(owner,response):
 def stopped():
  with owner.state_lock:return owner.health.estop and not owner.enabled and not owner.gripper_enabled and owner.board is not None and not owner.dry
 if not stopped():response.success=False;response.message='Need latched stop and both motion gates OFF';return response
 if not owner.feedback_cycle_lock.acquire(timeout=.8):response.success=False;response.message='Feedback transaction busy';return response
 measurements={}
 try:
  for name,(ident,*_) in JOINTS.items():
   if not stopped():raise RuntimeError('Stopped precondition changed')
   row={'servo_id':ident};measurements[name]=row
   for key,method in [('position_raw','position'),('torque_enabled','torque_enabled'),('voltage_mv','voltage_mv'),('temperature_C','temperature')]:
    if not stopped():raise RuntimeError('Stopped precondition changed')
    row[key]=getattr(owner.board,method)(ident);row[key+'_read_monotonic']=time.monotonic()
  response.success=True
 except Exception as error:response.success=False;measurements['read_error']=repr(error)
 finally:owner.feedback_cycle_lock.release()
 response.message=json.dumps({'joint_measurements':measurements,'serial_owner_reused':True,'motion_commands':0,'torque_writes':0,'transport':owner.board.diagnostics()})
 return response
'''
arm=(ROOT/'vla_sim_adapter/reference/arm_node.py').read_text(encoding='utf-8')
needle="        self.create_service(Trigger, '/arm_controller/reset_estop', self.reset)"
assert arm.count(needle)==1
arm=arm.replace(needle,needle+"\n        from .gripper_diag import read_diagnostics\n        self.create_service(Trigger, '/arm_controller/read_gripper_diagnostics', lambda req,res:read_diagnostics(self,res))")
proto=(ROOT/'vla_sim_adapter/reference/servo_protocol.py').read_text(encoding='utf-8')
needle="            self.events.append({'event': event, 'monotonic_time': time.monotonic(), **details})"
assert proto.count(needle)==1
proto=proto.replace(needle,needle+'''
            if event=='tx':
                raw=bytes.fromhex(details['frame_hex'])
                if len(raw)>=6 and raw[:3]==b'\\xaa\\x55\\x05' and raw[4] in (1,3):
                    if not hasattr(self,'last_control_tx'):self.last_control_tx={}
                    key='move' if raw[4]==1 else 'stop'
                    self.last_control_tx[key]={'monotonic_time':time.monotonic(),'frame_hex':details['frame_hex']}
''')
needle="return {'counters': dict(self.counters), 'last_query': copy.deepcopy(self.last_query),"
assert proto.count(needle)==1
proto=proto.replace(needle,"return {'last_control_tx': copy.deepcopy(getattr(self,'last_control_tx',{})), 'counters': dict(self.counters), 'last_query': copy.deepcopy(self.last_query),")
query=json.loads((ROOT/'logs/t07/runs/20261117T042848Z_6a8f5806/native_query_10618b20/stdout.log').read_text(encoding='utf-8'))
launch=query['launch_files']['base.launch.py'];needle="    cfg = os.path.join(get_package_share_directory('armpi_ros2'), 'config', 'base.yaml')"
assert launch.count(needle)==1
launch=launch.replace(needle,needle+"\n    readonly_lock=os.path.exists(os.path.join(get_package_share_directory('armpi_ros2'),'config','t07_readonly_lock.flag'))")
for name in ['enable_motion','enable_arm_motion','enable_gripper_motion']:
 old="ParameterValue(LaunchConfiguration('"+name+"'), value_type=bool)";new="(False if readonly_lock else "+old+")";assert old in launch;launch=launch.replace(old,new)
files={'gripper_diag.py':helper,'arm_node.py':arm,'servo_protocol.py':proto,'base.launch.py':launch}
for name,source in files.items():compile(source,name,'exec');(patch/name).write_bytes(source.encode())
sha=lambda b:hashlib.sha256(b).hexdigest()
proposal={'operator_approval_for_restart':'approved_existing_session_readonly_restart; continued authorized diagnostic repair, no torque/motion','latch_stop_before_restart':True,'baseline_arm_source_sha256':sha((ROOT/'vla_sim_adapter/reference/arm_node.py').read_bytes()),'baseline_launch_sha256':sha(query['launch_files']['base.launch.py'].encode()),'baseline_protocol_sha256':sha((ROOT/'vla_sim_adapter/reference/servo_protocol.py').read_bytes()),'module_filename':'gripper_diag.py','protocol_patch':True,'scope':'Read-only six-joint voltage/torque/position/temperature and last move/stop trace; startup gates OFF; native limits unchanged'}
(patch/'proposal.json').write_text(json.dumps(proposal,indent=2)+'\n',encoding='utf-8');print(json.dumps(proposal))
