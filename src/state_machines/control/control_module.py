import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

# from dao.location.getSceneFlow import getCtrlFlow #nav
from dao.velocity.getFlowByRobotList import getCtrlFlow # cmd_vel

class ControlRequest(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        self.ctrl_pub = rospy.Publisher('/scene_manager/ctrl_module_req', String, queue_size=1)

        self.request_robot_list = []

    def execute(self, user_data):
        rospy.sleep(0.1)

        full_cmd_list = str(user_data.command).split()

        self.request_robot_list = full_cmd_list[2:]

        ctrl_flow = getCtrlFlow(user_data.scene, self.request_robot_list)
        
        user_data.robot_list =[]

        if len(ctrl_flow) == 0:
            return 'none'
        
        while ctrl_flow:

            goal_data = ctrl_flow.popleft()

            self.ctrl_pub.publish(goal_data)

            rospy.loginfo("[CtrlModule] ctrl_module_req is published now.")
            rospy.loginfo("[CtrlModule] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])
        
        rospy.loginfo("[CtrlModule] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'
        
class InControl(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/ctrl_module_res', String, self.check_leftover,
                                        input_keys=['scene', 'robot_list'],
                                        output_keys=['robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        rospy.loginfo("[CtrlModule] result %s", str(result))

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s has completed control", result[0])
            rospy.loginfo("[CtrlModule] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False


class CtrlModuleSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        with self:
            self.add('CONTROL_REQUEST', ControlRequest(),
                     transitions={'done': 'IN_CONTROL',
                                  'none': 'complete'})
            self.add('IN_CONTROL', InControl(),
                     transitions={'invalid': 'complete',
                                'valid': 'IN_CONTROL',
                                'preempted':'IN_CONTROL'})
