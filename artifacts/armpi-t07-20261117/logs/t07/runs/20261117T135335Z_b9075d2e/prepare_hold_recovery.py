"""Hold existing arm positions while raising shoulder; retain all drift/arrival limits."""
import hashlib,json
from pathlib import Path
RUN=Path(__file__).resolve().parent
text=(RUN/'bounded_recovery.py').read_text(encoding='utf-8')
edits={
 "'t07_upward_reset_preparation'":"'t07_upward_reset_with_holds'",
 "msg.joint_names=['joint2'];pt=JointTrajectoryPoint();pt.positions=[(target-500)*math.pi/750]":"msg.joint_names=['joint1','joint2','joint3','joint4','joint5'];pt=JointTrajectoryPoint();command=dict(before);command['joint2']=target;pt.positions=[(command[k]-500)*math.pi/750*(-1 if k=='joint3' else 1) for k in msg.joint_names]",
 "ack.get('targets')!=[[5,target]]":"ack.get('targets')!=[[sid,command[k]] for k,sid in zip(msg.joint_names,[6,5,4,3,2])]",
 "'target_raw':target,'duration':3":"'target_raw':target,'held_raw':{k:before[k] for k in before if k not in ('joint2','r_joint')},'duration':3",
}
for old,new in edits.items():
 assert text.count(old)==1,old
 text=text.replace(old,new)
compile(text,'bounded_hold_recovery','exec')
(RUN/'bounded_hold.py').write_text(text,encoding='utf-8')
runner=(RUN/'run_recovery.py').read_text(encoding='utf-8').replace("RUN/'bounded_recovery.py'","RUN/'bounded_hold.py'")
runner=runner.replace('name=source.stem',"name='hold_'+source.stem").replace("name=='bounded_recovery'","name=='hold_bounded_hold'").replace("name=='t07_native_query'","name=='hold_t07_native_query'").replace("RUN/'post_recovery_camera.jpg'","RUN/'hold_post_recovery_camera.jpg'")
(RUN/'run_hold_recovery.py').write_text(runner,encoding='utf-8')
(RUN/'hold_preparation.json').write_text(json.dumps({'source_sha256':hashlib.sha256(text.encode()).hexdigest(),'reason':'First single-axis recovery stopped on elbow drift. This variant commands the other arm joints to hold their fresh initial positions. Drift tolerance3ticks, arrival6ticks, step20ticks/3seconds, temperatures and cleanup unchanged. No gripper or base command.','max_steps':3},indent=2))
print(json.dumps({'prepared':True,'source_sha256':hashlib.sha256(text.encode()).hexdigest()}))
