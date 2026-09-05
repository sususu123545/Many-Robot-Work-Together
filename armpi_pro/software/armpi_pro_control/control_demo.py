from action_group_controller import ActionGroupController
from ros_robot_controller_sdk import Board

board = Board()
controller = ActionGroupController(board)

# 动作组需要保存在当前路径的ActionGroups下
controller.runAction('wave_pro') # 参数为动作组的名称，不包含后缀，以字符形式传入
