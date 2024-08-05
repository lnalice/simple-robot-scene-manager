import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

# from helper.getSceneFlow import getMoveFLow #nav
from helper.getSceneFlowVel import getMoveFLow # cmd_vel

class CtrlModuleSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["done"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
