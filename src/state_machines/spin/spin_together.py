import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

# from dao.getSceneFlow import getMoveFLow # nav
from dao.getSceneFlowVel import getMoveFLow # cmd_vel

DISPLAY_TIME = 10

class SpinRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):

        move_flow = getMoveFLow("turn_around")
        scene_flow = getMoveFLow(user_data.scene)
        goal_data = move_flow.popleft()

        user_data.robot_list = []

        rospy.sleep(DISPLAY_TIME)
        rospy.loginfo("[SpinTogether] I'm waiting for the exhibition time to end.(time: %s)\n", DISPLAY_TIME)

        while scene_flow:
            
            scene_data = scene_flow.popleft().split()
            robot_id = scene_data[0]
            
            self.move_pub.publish(robot_id + goal_data)

            rospy.loginfo("[SpinTogether] move_req is published now. robot_id %s", robot_id)
            rospy.loginfo("[SpinTogether] data published now: %s", robot_id + goal_data)

            user_data.robot_list.append(robot_id)

        rospy.loginfo("[SpinTogether] robot_list is updated now (%s)", str(user_data.robot_list))

        if len(user_data.robot_list) == 0:
            return 'none'

        return 'done'

class Spin(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/move_res', String, self.check_leftover,
                                        input_keys=['scene', 'robot_list'],
                                        output_keys=['scene', 'robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s arrived", result[0])
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False

class Arrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
    def execute(self, user_data):
        #[todo] error handling (fail)
        return 'done'            

class SpinTogetherSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])

        with self:
            self.add('SPIN_REQUEST', SpinRequest(),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': 'arrive'})
            self.add('ON_THE_MOVE', Spin(),
                     transitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })