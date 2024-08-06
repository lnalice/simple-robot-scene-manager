import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

# from helper.getSceneFlow import getCtrlFlow #nav
from helper.getSceneFlowVel import getCtrlFlow # cmd_vel

class ControlRequest(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["done"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        self.ctrl_pub = rospy.Publisher('/scene_manager/ctrl_module_req', String, queue_size=1)

    def execute(self, user_data):
        rospy.sleep(0.1)

        ctrl_flow = getCtrlFlow(user_data.scene)
        
        user_data.robot_list =[]

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
                                        output_keys=['scene', 'robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s has completed control", result[0])
            rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False


class CtrlModuleSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        with self:
            self.add('CONTROL_REQUEST', ControlRequest(),
                     transitions={'done': 'IN_CONTROL'})
            self.add('IN_CONTROL', InControl(),
                     transitions={'invalid': 'complete',
                                'valid': 'IN_CONTROL',
                                'preempted':'IN_CONTROL'})
