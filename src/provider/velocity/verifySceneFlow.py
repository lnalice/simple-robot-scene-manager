import os
import json

json_vel_rel_loc = 'json/scene_param_vel.json'
base_path = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))  # base path: scene_manager/src/
json_vel_loc = os.path.join(base_path, json_vel_rel_loc) 

"""
** verifiedRobotList **
this returns robot_list included in the scene

- return all robot_id in the scene, if @param roobt_list is empty
   ([] -> all robot ids in the scene)
"""
def verifiedRobotList(scene:str, robot_list:list) -> list:
    
    verified_robot_list = []

    all: bool = len(robot_list) == 0

    base_path = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    json_vel_loc = os.path.join(base_path, 'json/scene_param_vel.json') 

    print(json_vel_loc)
    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]

                if all or id in robot_list:
                     verified_robot_list.append(id)

    return verified_robot_list