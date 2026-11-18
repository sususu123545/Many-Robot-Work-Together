"""Choose actual temperature observations without modifying any joint state."""
import math
TEMPERATURE_SERVO_IDS={'joint1':6,'joint2':5,'joint3':4,'joint4':3,'joint5':2,'r_joint':1}
def observed_temperature(name,status_row,observation,now):
 result={'value':status_row.get('temperature',999),'time':status_row.get('temperature_time',0),'source':'native_status'}
 if not isinstance(observation,dict):return result
 try:
  value=observation['temperature_C'];stamp=observation['temperature_time']
  valid=(observation['source']=='physical_temperature_read' and observation['joint']==name
         and observation['servo_id']==TEMPERATURE_SERVO_IDS[name]
         and observation.get('retained') is False and math.isfinite(value) and math.isfinite(stamp)
         and 0<=value<=100 and 0<=now-stamp<1.5 and stamp>=result['time'])
 except (KeyError,TypeError,ValueError):return result
 if valid:return {'value':value,'time':stamp,'source':'physical_temperature_read'}
 return result
