import rospy
import smach
import smach_ros

from std_msgs import String

class MoveRequest(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["done"],
                                    input_keys=['robot_name', 'goal_pose'],
                                    output_keys=['robot_name'])
        
        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):

        # id pos(x,y) orientation(z,w)
        orientation = user_data.goal_pose.orientation
        position = user_data.goal_pose.position
        
        goal_data = "%f %f %f %f" % (position.x, position.y, orientation.z, orientation.w)

        self.move_pub.publish(goal_data)

        return 'done'


class MoveSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['robot_name', 'goal_pose'],
                                    output_keys=['robot_name'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest,
                     transitions={'done', 'ON_THE_MOVE'})
            self.add('ON_THE_MOVE', OnTheMove,
                     trasitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive,
                     transitions={
                         'done': 'done',
                         'fail': 'MOVE_REQUEST'
                     })
