import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

from dao.RobotDao import updateRobotCurrentStatus, moduleStateByRobotID, displacementByRobotID #MySQL

from state_machines.move.move_together import OnTheMove #MonitorState (move)
from state_machines.control.control_module import InControl #MonitorState (control module)

from helper.moduleCalculator import moduleState2deg

"""
1. 상판 모듈 모두 최소화 (원래 상태로 복귀)
2. 원래 자기 위치로 복귀 명령 (모듈 최소화가 모두 완료된 후 이동)
"""
GO_HOME_SECONDS = 15
INTERVAL_SECONDS = 3
class ResetCtrl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        
        self.ctrl_pub = rospy.Publisher('/scene_manager/ctrl_module_req', String, queue_size=1)

        self.request_robot_list = []

    def execute(self, user_data):
        ctrl_flow = deque()

        full_cmd_list = str(user_data.command).split()
        self.request_robot_list = full_cmd_list[2:]

        for robotID in self.request_robot_list:
            (moduleState,) = moduleStateByRobotID(robotID)
            (degZ, degX) = moduleState2deg(moduleState)

            module_msg = "%s %f %f %d" %(robotID, degZ, degX, 0) #delay = 0
            ctrl_flow.append(module_msg)

        if len(ctrl_flow) == 0:
            return 'none'
        
        user_data.robot_list = []
        
        while ctrl_flow:
            goal_data = ctrl_flow.popleft()
            self.ctrl_pub.publish(goal_data)

            rospy.loginfo("[ResetCtrl] ctrl_module_req is published now.")
            rospy.loginfo("[ResetCtrl] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])
        
        rospy.loginfo("[ResetCtrl] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'

class ResetMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

        self.request_robot_list = []

    def execute(self, user_data):
        sorted_list = []

        full_cmd_list = str(user_data.command).split()
        self.request_robot_list = full_cmd_list[2:]

        for robotID in self.request_robot_list:
            displacementX, displacementZ = displacementByRobotID(robotID)

            linX = displacementX / GO_HOME_SECONDS
            angZ = displacementZ / GO_HOME_SECONDS

            move_msg = "%s %d %f 0 0 0 0 %f %f" %(robotID, GO_HOME_SECONDS, linX, angZ, 0) #delay = 0
            sorted_list.append(move_msg)

        sorted_list.sort(key=lambda x: x.split()[2])
        move_flow = deque(sorted_list)
        
        user_data.robot_list = []

        if len(move_flow) == 0:
            return 'none'
        
        while move_flow:
            goal_data = move_flow.popleft() 

            self.move_pub.publish(goal_data)

            rospy.loginfo("[ResetMove] move_req is published now.")
            rospy.loginfo("[ResetMove] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])
            rospy.sleep(INTERVAL_SECONDS)
        
        rospy.loginfo("[ResetMove] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'

class ResetSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])

        with self:
            self.add('CONTROL_REQUEST', ResetCtrl(),
                     transitions={'done': 'IN_CONTROL',
                                  'none': 'MOVE_REQUEST'})
            self.add('IN_CONTROL', InControl(),
                     transitions={'invalid': 'MOVE_REQUEST',
                                'valid': 'IN_CONTROL',
                                'preempted':'IN_CONTROL'})
            self.add('MOVE_REQUEST', ResetMove(),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': "complete"})
            self.add('ON_THE_MOVE', OnTheMove(),
                       transitions={'invalid': 'complete',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            