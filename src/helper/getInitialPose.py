from collections import deque
import json


json_loc = '/home/domain/robot_ws/src/robot_planner/src/data/initial_pose.json'

def getInitialPose(robot_id:str) -> tuple:
    # read json file
    with open(json_loc) as json_file:
            robot_init = json.load(json_file)[robot_id]
            
            position =robot_init[position]
            orientation = robot_init[orientation]
            
            return (position["x"], position["y"], orientation["z"], orientation["w"])
    