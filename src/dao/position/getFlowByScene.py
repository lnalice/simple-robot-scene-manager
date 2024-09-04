import os
from collections import deque
import json

json_vel_rel_loc = 'data/scene_param_mdl.json'
base_path = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))  # base path: scene_manager/src/
json_vel_loc = os.path.join(base_path, json_vel_rel_loc) 

def getCtrlFlow(scene:str) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                module = robot_goal["module_pos"]

                if module["z"] == 0 and module["x"] == 0:
                     continue
                
                delay_sec = robot_goal["delay"]
                
                task = "%s %f %f %d" %(id, module["z"], module["x"], delay_sec)

                ctrl_flow.append(task)

    return ctrl_flow

"""
** getOppositeDegreeFLow **
this SELECT function is for 'go home'
- return opposit direction from scene
- return non-delay seconds
"""
def getOppositeDegreeFLow(scene:str) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                module = robot_goal["module_pos"]

                if module["z"] == 0 and module["x"] == 0:
                     continue
                
                delay_sec = robot_goal["delay"]
                
                task = "%s %f %f %d" %(id, -module["z"], -module["x"], delay_sec)

                ctrl_flow.append(task)

    return ctrl_flow