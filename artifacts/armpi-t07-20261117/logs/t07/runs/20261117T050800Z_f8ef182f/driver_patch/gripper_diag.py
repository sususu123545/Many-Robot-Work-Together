"""Read through the existing bus owner only, under latched stop and OFF gates."""
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
