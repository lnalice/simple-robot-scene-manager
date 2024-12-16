import rospy
import smach
import smach.state
import smach_ros
from std_msgs.msg import String

import queue

"""
@request    {taskID} {role1} {# of role1} {role2} {# of role2}
"""
class RequestMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/react/commander', String, self.check_command,
                                        input_keys=['context', 'roleData', 'context_queue'],
                                        output_keys=['context', 'roleData', 'context_queue'])
        
    def check_command(self, user_data, res_msg):
        full_cmd = str(res_msg.data)

        cmd_list = full_cmd.split()
        user_data.context = cmd_list[0]
        data = cmd_list[1:]

        roleData = {}
        for i in range(0, len(data), 2):
            roleData[data[i]] = int(data[i + 1])

        user_data.roleData = roleData

        rospy.loginfo(f"[ContextMonitor] I got new command for %s.", user_data.context)
        
        return False

class WriteContext(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                             io_keys=['context', 'roleData', 'context_queue'])

    def execute(self, user_data):
        if user_data.context == "":
            return 'done'
        
        context_queue: queue.Queue = user_data.context_queue
        context_queue.put((user_data.context,user_data.roleData))

        rospy.loginfo(f"[ContextMonitor] I put new request into the queue. ({user_data.context})")

        # reset
        user_data.context = "" 
        user_data.roleData = {}

        return 'done'

class ContextMonitorSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['done'],
                                    input_keys=['context_queue'],
                                    output_keys=['context_queue'])

        self.userdata.context = "" # ex) FRIEND
        self.userdata.roleData = {} # ex) { 'table': 10, 'chair': 15 }

        with self:
            smach.StateMachine.add('WAIT_CONTEXT', RequestMonitor(),
                                   transitions={'invalid': 'WRITE_CONTEXT',
                                                'valid': 'WAIT_CONTEXT',
                                                'preempted':'WAIT_CONTEXT'})
            smach.StateMachine.add('WRITE_CONTEXT', WriteContext(),
                                   transitions={'done': 'WAIT_CONTEXT'})
        