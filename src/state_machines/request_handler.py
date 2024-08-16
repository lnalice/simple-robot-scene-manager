import rospy
import smach
import smach.state
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

"""
@request    SCENE  {scene_id} {robot_id} {robot_id
@request    MOVE   {scene_id} {robot_id} {robot_id}...
@request    MODULE {scene_id} {robot_id} {robot_id}...
@request    HOME   {scene_id} {robot_id} {robot_id}...
"""
class RequestMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/react/commander', String, self.check_command,
                                        input_keys=['command'],
                                        output_keys=['command'])
        
    def check_command(self, user_data, res_msg):
        user_data.command = str(res_msg.data)

        result = str(res_msg.data).split()

        rospy.loginfo(f"[RequestInterpreter] I got new command for %s.", result[0])
        
        return False
    
class Request2State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scene', 'move', 'module', 'home'],
                             input_keys=['scene', 'robot_list', 'command'],
                             output_keys=['scene', 'robot_list', 'command'])
        
    def execute(self, user_data):
        full_cmd_list = str(user_data.command).split()

        user_data.command = full_cmd_list[0]
        user_data.scene = full_cmd_list[1]
        user_data.robot_list = full_cmd_list[2:]

        rospy.loginfo(f"[RequestInterpreter] I saved state %s in SceneManager", user_data.command)
        
        if user_data.command == "MOVE":
            return "move"
        elif user_data.command == "HOME":
            return "home"
        elif user_data.command == "MODULE":
            return "module"
        else:
            return "scene"

class RequestInterpreterSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['scene', 'move', 'module', 'home'],
                                    input_keys=['command'],
                                    output_keys=['command'])
        
        with self:
            self.add('REQUEST', RequestMonitor(),
                     transitions={'invalid': 'INTERPRET',
                                  'valid': 'REQUEST',
                                  'preempted':'REQUEST'})
            self.add('INTERPRET', Request2State())