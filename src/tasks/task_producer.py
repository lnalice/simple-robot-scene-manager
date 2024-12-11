import rospy
import smach
import smach.state
import smach_ros
from std_msgs.msg import String

import queue

"""
@request    MOVE   {scene_id} {robot_id} {robot_id}...
@request    MODULE {scene_id} {robot_id} {robot_id}...
@request    MIN    {scene_id} {robot_id} {robot_id}...
@request    HOME   {scene_id} {robot_id} {robot_id}...
"""
class RequestMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, 'react/commander', String, self.check_command,
                                        input_keys=['command', 'sync_queue'],
                                        output_keys=['command', 'sync_queue'])
        
    def check_command(self, user_data, res_msg):
        user_data.command = str(res_msg.data)

        result = user_data.command.split()

        # verify new command

        rospy.loginfo(f"[TaskProducer] I got new command for %s.", result[0])
        
        return False

class WriteTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                             input_keys=['command', 'sync_queue'],
                             output_keys=['command', 'sync_queue'])

    def execute(self, user_data):
        if user_data.command == "":
            return 'done'
        
        sync_queue: queue.Queue = user_data.sync_queue
        sync_queue.put(user_data.command)
        rospy.loginfo(f"[TaskProducer] I put new request into the queue. ({user_data.command})")

        user_data.command = "" # reset command

        return 'done'

class TaskProducerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['done'],
                                    input_keys=['sync_queue'],
                                    output_keys=['sync_queue'])

        self.userdata.command = ""

        with self:
            smach.StateMachine.add('WAIT_TASK', RequestMonitor(),
                                   transitions={'invalid': 'WRITE_TASK',
                                                'valid': 'WAIT_TASK',
                                                'preempted':'WAIT_TASK'})
            smach.StateMachine.add('WRITE_TASK', WriteTask(),
                                   transitions={'done': 'WAIT_TASK'})
        