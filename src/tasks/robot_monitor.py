import smach
from dao.trigger.binlog import robot_event_cb, bitlog_stream

class RobotMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['updated', 'none', 'fail'],
                             io_keys=['idle_robots'])
        
        self.stream = bitlog_stream

    def execute(self, user_data):
        try:
            for event in self.stream:
                result = robot_event_cb(user_data.idle_robots, event)
        except:
            return 'fail'
        
        return 'updated' if result else 'none'
        