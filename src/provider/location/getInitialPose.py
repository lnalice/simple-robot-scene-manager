import os
from collections import deque
import json

json_rel_loc = 'json/scene_param.json'
base_path = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))  # base path: scene_manager/src/
json_loc = os.path.join(base_path, json_rel_loc) 

def getInitialPose(robot_id:str) -> tuple:
    # read json file
    with open(json_loc) as json_file:
            robot_init = json.load(json_file)[robot_id]
            
            position =robot_init[position]
            orientation = robot_init[orientation]
            
            return (position["x"], position["y"], orientation["z"], orientation["w"])
    