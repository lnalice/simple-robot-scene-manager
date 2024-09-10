import rospy
import smach
import smach.state
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from provider.velocity.verifySceneFlow import verifiedRobotList

"""
@request    SCENE  {scene_id} {robot_id} {robot_id
@request    MOVE   {scene_id} {robot_id} {robot_id}...
@request    MODULE {scene_id} {robot_id} {robot_id}...
@request    HOME   {scene_id} {robot_id} {robot_id}...
"""
class RequestMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, 'react/commander', String, self.check_command,
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

        action = full_cmd_list[0] # MOVE or HOME or MODULE or SCENE
        user_data.scene = full_cmd_list[1] # ex) scene_1, module_50
        if action == "SCENE":
            user_data.robot_list = verifiedRobotList(user_data.scene, full_cmd_list[2:]) # ex) ['tb3_0', 'tb3_1', 'tb3_2']
        else:
            user_data.robot_list = full_cmd_list[2:]

        rospy.loginfo(f"[RequestInterpreter] SceneManager will track these robots: %s.", (user_data.robot_list))
        rospy.loginfo(f"[RequestInterpreter] I saved state \'%s\' in SceneManager", action)
        
        if action == "MOVE":
            return "move"
        elif action == "HOME":
            return "home"
        elif action == "MODULE":
            return "module"
        else:
            return "scene"

class RequestInterpreterSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['scene', 'move', 'module', 'home'],
                                    input_keys=['scene', 'robot_list', 'command'],
                                    output_keys=['scene', 'robot_list', 'command'])
        
        with self:
            self.add('REQUEST', RequestMonitor(),
                     transitions={'invalid': 'INTERPRET',
                                  'valid': 'REQUEST',
                                  'preempted':'REQUEST'})
            self.add('INTERPRET', Request2State())