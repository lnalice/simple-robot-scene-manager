import json

json_vel_loc = '/home/domain/robot_ws/src/scene_manager/src/data/scene_param_vel.json'

"""
** verifiedRobotList **
this returns robot_list included in the scene

- return all robot_id in the scene, if @param roobt_list is empty
   ([] -> all robot ids in the scene)
"""
def verifiedRobotList(scene:str, robot_list:list) -> list:
    
    verified_robot_list = []

    all: bool = len(robot_list) == 0

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]

                if all or id in robot_list:
                     verified_robot_list.append(id)

    return verified_robot_list