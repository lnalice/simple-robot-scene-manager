import rospy
import smach
import smach_ros

import argparse

from helper.getSceneFlow import getSceneFlow
from state_machines.move import MoveTogetherSM
                
class ScenePlanner:
    def __init__(self, param):

        rospy.init_node("scene_planner_node")

        # robot_id goal_pose.position(x,y) goal_pose.orientation(z,w)
        self.scene = "scene" + "_" + param.scene

        self.moveFlow, self.ctrlFLow = getSceneFlow(self.scene)

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.userdata.scene = self.scene
        self.sm.userdata.robot_list = []

        with self.sm:
            smach.StateMachine.add('MOVE', MoveTogetherSM(),
                                   input_keys=['scene', 'robot_list'], output_keys=['scene', 'robot_list'],
                                   transition={ 
                                       'done': 'TASK_ALLOCATION'})
                                    #    'end': 'CTRL_MODULE'})
            # smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(self.ctrl_task_id),
            #                        transision={'done': 'CTRL_MODULE',
            #                                    'end':'COME_BACK'})
            # smach.StateMachine.add("COME_BACK",)


parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-s', '--scene', required=True, nargs='+', help='Scene Number (1, 2, 3)') # ex) 1 3 or 1
args = parser.parse_args()

if __name__ == "__main__":
    simple_traveler = ScenePlanner(param=args)
    simple_traveler.move_action()
    rospy.spin()