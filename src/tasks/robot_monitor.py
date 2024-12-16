import rospy
import smach
import smach_ros

from std_msgs.msg import String

def monitor_cb(user_data, res_msg):
    result = str(res_msg.data).split() #{robot} {status} (ex. tb3_0 IDLE)

    robot, status = result

    if status == 'IDLE':
        user_data.idle_robots.add(robot)
    else:
        user_data.idle_robots.discard(robot)
    
    rospy.loginfo(f"Robot {robot}'s status is updated to {status}.")

    return True

class RobotMonitorSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['updated'],
                             input_keys=['idle_robots'],
                             output_keys=['idle_robots'])
        
        self.monitor = smach_ros.MonitorState("/task_scheduler/robot_status", String, monitor_cb,)

        with self:
            self.add('INIT', self.monitor,
                     transitions = {'done': 'MONITOR'})
            self.add('MONITOR', self.monitor,
                     transitions = {'invalid': 'MONITOR',
                                   'valid': 'MONITOR',
                                   'preempted': 'MONITOR'})