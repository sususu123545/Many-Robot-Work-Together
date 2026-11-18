"""Prepared one-shot servo-1 restore; requires an explicit consumed permit file."""
import json,time
from pathlib import Path

def restore_once(owner,response,permit_path):
    values={'servo_id':1,'torque_writes':0,'holding_target_commands':0}
    def stopped():
        with owner.state_lock:return owner.health.estop and not owner.enabled and not owner.gripper_enabled and owner.board is not None and not owner.dry
    permit=Path(permit_path)
    if not permit.is_file() or not stopped():
        response.success=False;response.message='Missing one-shot authorization or stopped-state preconditions';return response
    if not owner.feedback_cycle_lock.acquire(timeout=.8):
        response.success=False;response.message='Feedback busy; no write';return response
    try:
        if not stopped():raise RuntimeError('Stopped-state changed')
        b=owner.board
        if b.torque_enabled(1):raise RuntimeError('Torque already enabled; duplicate restore refused')
        points=[b.position(1) for _ in range(3)];lo,hi=b.limits(1);raw=sorted(points)[1]
        if max(points)-min(points)>3 or not lo<=raw<=hi or not 0<=raw<=700:raise RuntimeError('Position unstable or outside measured limits')
        if b.temperature(1)>=min(40,owner.temp_max):raise RuntimeError('Temperature guard')
        values.update(initial_raw=raw,voltage_mv=b.voltage_mv(1),initial_torque=False)
        if not stopped():raise RuntimeError('Stopped-state changed')
        permit.unlink()  # consume before any write; errors never automatically retry
        b.move(1.0,[(1,raw)]);values['holding_target_commands']=1
        if b.torque_enabled(1):raise RuntimeError('Unexpected torque activation while preloading hold target')
        if abs(b.position(1)-raw)>3 or not stopped():raise RuntimeError('Position or stopped-state changed before torque write')
        with b.lock:b._write(bytes((0x0B,1)))
        values['torque_writes']=1;time.sleep(.2)
        values.update(final_torque=bool(b.torque_enabled(1)),final_raw=b.position(1))
        if not values['final_torque'] or abs(values['final_raw']-raw)>3:
            b.stop([1]);values['targeted_stop_sent']=True;raise RuntimeError('Torque restore or holding position verification failed')
        response.success=True;response.message=json.dumps({'passed':True,**values,'arm_and_base_commands':0,'offset_or_limit_writes':0})
    except Exception as e:
        response.success=False;response.message=json.dumps({'passed':False,'error':str(e),**values})
    finally:owner.feedback_cycle_lock.release()
    return response
