import rospy
import smach

from std_msgs.msg import String

# queue size를 정의할 수 있는 custome MonitorState
class RobotMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['received', 'none'],
                            io_keys=['idle_robots'])
                             
        self.res_data = None
    
    def monitor_cb(self, res_msg):
        self.res_data = res_msg.data

    def execute(self, user_data):
        rospy.Subscriber("/task_scheduler/robot_status", String, self.monitor_cb, queue_size=10)
        rospy.sleep(1)

        while True:
            if self.res_data:
                result = str(self.res_data).split() # {robot} {status} (ex. tb3_0 IDLE)
                robot, status = result

                if status == 'IDLE':
                    user_data.idle_robots.add(robot)
                else:
                    user_data.idle_robots.discard(robot)
                
                rospy.loginfo(f"[RobotMonitor] Robot {robot}'s status is updated to {status}.")

                self.res_data = None
                return 'received'
    

class RobotMonitorSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['updated'],
                             input_keys=['idle_robots'],
                             output_keys=['idle_robots'])
        
        with self:
            self.add('MONITOR', RobotMonitor(),
                     transitions = {'received': 'MONITOR',
                                   'none': 'MONITOR'})