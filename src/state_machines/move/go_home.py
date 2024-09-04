import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

DISPLAY_TIME = 10.0

class HomeRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'robot_list'],
                                    output_keys=['robot_list'])
        
        self.go_home_pub = rospy.Publisher('/scene_manager/go_home', String, queue_size=1)
        self.request_robot_list = []

    def execute(self, user_data):
        move_flow = deque()

        full_cmd_list = str(user_data.command).split()

        display_time = DISPLAY_TIME if full_cmd_list[0] == "SCENE" else 0

        self.request_robot_list = full_cmd_list[2:]

        user_data.robot_list = []
        
        for robot in self.request_robot_list:
            rospy.sleep(0.1)
            
            self.go_home_pub.publish(robot)

            rospy.loginfo("[GoHome] move_req is published now.")
            rospy.loginfo("[GoHome] data published now: %s", robot)

            user_data.robot_list.append(robot)
        
        rospy.loginfo("[GoHome] robot_list is updated now (%s)", str(user_data.robot_list))

        if len(user_data.robot_list) == 0:
            return 'none'

        return 'done'

class OnTheMove(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/come_back_home', String, self.check_leftover,
                                        input_keys=['command', 'robot_list'],
                                        output_keys=['robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s arrived", result[0])
            rospy.loginfo("[GoHome] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False

class Arrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, user_data):
        #[todo] error handling (fail)
        return 'done'            

class GoHomeSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['command', 'robot_list'],
                                    output_keys=['robot_list'])
        
        with self:
            self.add('HOME_REQUEST', HomeRequest(),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': "arrive"})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })
