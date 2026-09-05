import threading, os, time
from ros_robot_controller_sdk import Board
from bus_servo_control import BusServoControl
from action_group_controller import ActionGroupController

board = Board()

agc = ActionGroupController(board)
bsc = BusServoControl(board)

def getServoPulse(servo_id):
    data = bsc.getBusServoPulse(servo_id)
    if data is not None:
        return data[0]
    else:
        return None

def getServoDeviation(servo_id):
    data = bsc.getBusServoDeviation(servo_id)
    if data is not None:
        return data[0]
    else:
        return None

def setServoPulse(servo_id, pulse, use_time):
    bsc.setBusServoPulse(servo_id, pulse, use_time)

def setServoDeviation(servo_id ,dev):
    bsc.setBusServoDeviation(servo_id, dev)
    
def saveServoDeviation(servo_id):
    bsc.saveBusServoDeviation(servo_id)

def unloadServo(servo_id):
    bsc.unloadBusServo(servo_id)

def runActionGroup(num):
    threading.Thread(target=agc.runAction, args=(num, )).start()    

def stopActionGroup():    
    agc.stop_action_group()

def enable_reception(enable=True):
    if enable:
        board.enable_reception(not enable)
        time.sleep(1)
        threading.Thread(target=os.system, args=("/bin/zsh -c \'source $HOME/armpi_pro/src/armpi_pro_bringup/scripts/source_env.bash && rostopic pub /ros_robot_controller/enable_reception std_msgs/Bool \"data: true\"\'",), daemon=True).start()
        time.sleep(1)
    else:
        threading.Thread(target=os.system, args=("/bin/zsh -c \'source $HOME/armpi_pro/src/armpi_pro_bringup/scripts/source_env.bash && rostopic pub /ros_robot_controller/enable_reception std_msgs/Bool \"data: false\"\'",), daemon=True).start()
        time.sleep(3)
        board.enable_reception(not enable)
        time.sleep(1)

