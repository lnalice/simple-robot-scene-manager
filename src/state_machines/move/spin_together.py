import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

from helper.getSceneFlowVel import getMoveFLow # cmd_vel

class SpinRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):
        move_flow = getMoveFLow("turn_around")
        scene_flow = getMoveFLow(user_data.scene)
        goal_data = move_flow.popleft()

        while scene_flow:
            rospy.sleep(1)

            scene_data = scene_flow.popleft().split()
            robot_id = scene_data[0]
            
            self.move_pub.publish(robot_id + goal_data)

            rospy.loginfo("[MoveTogether] move_req is published now.")
            rospy.loginfo("[MoveTogether] data published now: %s", goal_data)

            user_data.robot_list.append(robot_id)

        rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))

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
            return False
        
        return True

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
                     transitions={'done': 'ON_THE_MOVE'})
            self.add('ON_THE_MOVE', Spin(),
                     transitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })
