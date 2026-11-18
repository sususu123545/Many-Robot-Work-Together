"""Attach actual temperature observations to safety checks, never joint-state data."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def attach_temperature_observer(prefix):
 prefix=prefix.replace("now-j.get('temperature_time',0)","now-control_temperature(name,a)['time']").replace("j.get('temperature',999)","control_temperature(name,a)['value']")
 return prefix+'\n'+(ROOT/'scripts/t07_temperature_observer.py').read_text(encoding='utf-8')+'''
temperature_observations={}
def receive_temperature(message):
 try:
  data=json.loads(message.data);name=data.get('joint');now=time.monotonic()
  if name not in TEMPERATURE_SERVO_IDS:return
  candidate=observed_temperature(name,{},data,now)
  if candidate['source']!='physical_temperature_read':return
  previous=temperature_observations.get(name)
  if previous is None or data['temperature_time']>=previous['temperature_time']:temperature_observations[name]=data
  events.append({'kind':'physical_temperature_read','receive_monotonic':now,'data':data})
 except (TypeError,ValueError,KeyError):return
n.create_subscription(String,'/arm_controller/temperature_feedback',receive_temperature,20)
def control_temperature(name,status=None):
 a=latest.get('arm',{}) if status is None else status
 return observed_temperature(name,a.get('joints',{}).get(name,{}),temperature_observations.get(name),time.monotonic())
'''
