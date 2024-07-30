import rospy
import smach
import smach_ros

from std_msgs import String

class MoveRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                                    input_keys=['robot_id', 'goal_pose'],
                                    output_keys=['robot_id'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):
        # id position(x,y) orientation(z,w)
        position = user_data.goal_pose.position
        orientation = user_data.goal_pose.orientation
        
        goal_data = "%s %f %f %f %f" % (user_data.robot_id, position.x, position.y, orientation.z, orientation.w)

        self.move_pub.publish(goal_data)

        return 'done'

class OnTheMove(smach.State):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/move_res', String, self.check_validity,
                                        input_keys=['robot_id', 'goal_pose'],
                                        output_keys=['robot_id'])
        
    def check_validity(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] == user_data.robot_id:
            return False
        return True

class Arrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                    input_keys=['robot_id', 'goal_pose'],
                                    output_keys=['robot_id'])
    def execute(self, user_data):
        #error handling (fail)
        return 'done'            

class MoveSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['robot_id', 'goal_pose'],
                                    output_keys=['robot_id'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest(),
                     transitions={'done', 'ON_THE_MOVE'})
            self.add('ON_THE_MOVE', OnTheMove(),
                     trasitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })
