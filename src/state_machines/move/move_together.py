import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

from helper.getSceneFlow import getMoveFLow

class MoveRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['scene', 'robot_list'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=5)

    def execute(self, user_data):
        move_flow = getMoveFLow(user_data.scene)

        user_data.robot_list = []

        while move_flow:
            # task: {"robot_id": id, "pos_x": position.x, "pos_y": position.y, "ort_z": orientation.z, "ort_w":orientation.w}
            task = move_flow.popleft() 
            goal_data = "%s %f %f %f %f" %(task["robot_id"], task["pos_x"], task["pos_y"], task["ort_z"], task["ort_w"])

            self.move_pub.publish(goal_data)

            user_data.robot_list.append(task["robot_id"])
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
