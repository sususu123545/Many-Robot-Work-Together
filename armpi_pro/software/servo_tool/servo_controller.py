import threading, os, time
from ros_robot_controller_sdk import Board
from bus_servo_control import BusServoControl

board = Board()
# board.enable_reception()

bsc = BusServoControl(board)

def setServoID(old, new_servo_id):
    bsc.setBusServoID(old, new_servo_id)

def setServoPulse(servo_id, pulse, use_time):
    bsc.setBusServoPulse(servo_id, pulse, use_time)

def setServoDeviation(servo_id ,dev):
    bsc.setBusServoDeviation(servo_id, dev)

def setServoMaxTemp(servo_id, temp):
    bsc.setBusServoMaxTemp(servo_id, temp)

def setServoAngleLimit(servo_id, low, high):
    bsc.setBusServoAngleLimit(servo_id, low, high)

def setServoVinLimit(servo_id, low, high):
    bsc.setBusServoVinLimit(servo_id, low, high)

def getServoID(servo_id=None):
    return bsc.getBusServoID(servo_id)

def getServoPulse(servo_id):
    return bsc.getBusServoPulse(servo_id)

def getServoDeviation(servo_id):
    return bsc.getBusServoDeviation(servo_id)

def saveServoDeviation(servo_id):
    bsc.saveBusServoDeviation(servo_id)

def getServoTempLimit(servo_id):
    return bsc.getBusServoTempLimit(servo_id)

def getServoAngleLimit(servo_id):
    return bsc.getBusServoAngleLimit(servo_id)

def getServoVinLimit(servo_id):
    return bsc.getBusServoVinLimit(servo_id)

def getServoVin(servo_id):
    return bsc.getBusServoVin(servo_id)

def getServoTemp(servo_id):
    return bsc.getBusServoTemp(servo_id)

def unloadServo(servo_id):
    bsc.unloadBusServo(servo_id)

def enable_reception(enable=True):
    if enable:
        board.enable_reception(not enable)
        time.sleep(1)
        threading.Thread(target=os.system, args=("/bin/zsh -c \'source $HOME/armpi_pro/src/armpi_pro_bringup/scripts/source_env.bash && rostopic pub /ros_robot_controller/enable_reception std_msgs/Bool \"data: true\"\'",), daemon=True).start()
        time.sleep(1)
    else:
        threading.Thread(target=os.system, args=("/bin/zsh -c \'source $HOME/armpi_pro/src/armpi_pro_bringup/scripts/source_env.bash && rostopic pub /ros_robot_controller/enable_reception std_msgs/Bool \"data: false\"\'",), daemon=True).start()
        time.sleep(4)
        board.enable_reception(not enable)
        time.sleep(1)

