import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

# from helper.getSceneFlow import getMoveFLow #nav
from helper.getSceneFlowVel import getMoveFLow # cmd_vel

class MoveRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):
        rospy.sleep(0.1)
        
        move_flow = getMoveFLow(user_data.scene)

        user_data.robot_list = []

        while move_flow:
            
            goal_data = move_flow.popleft() 
            
            self.move_pub.publish(goal_data)

            rospy.loginfo("[MoveTogether] move_req is published now.")
            rospy.loginfo("[MoveTogether] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])
        
        rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'

class OnTheMove(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/move_res', String, self.check_leftover,
                                        input_keys=['scene', 'robot_list'],
                                        output_keys=['scene', 'robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s arrived", result[0])
            rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))
        
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

class MoveTogetherSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest(),
                     transitions={'done': 'ON_THE_MOVE'})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })
