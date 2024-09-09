from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

cnx = connect_to_mysql(mysql_config, attempts=3)

def selectMoveDataByScene (scene: str, isOpposite: bool, robot_list: list) -> deque:
    move_flow = deque()

    result = cnx.cursor(buffered=True)
    query = (
        "SELECT robotID, seconds, linX, angZ, delay FROM SceneMove "
        "WHERE sceneID = %s"
    )
    result.execute(query, [scene])

    for (robotID, seconds, linX, angZ, delay) in result:
        task =""

        if robotID not in robot_list:
            continue

        if linX == 0 and angZ == 0:
            continue

        if isOpposite:
            task = "%s %d %f 0 0 0 0 %f %f" %(robotID, seconds, -linX, angZ, delay)
        else:
            task = "%s %d %f 0 0 0 0 %f %f" %(robotID, seconds, linX, angZ, delay)
        move_flow.append(task)

    cnx.close()

    return move_flow


if __name__ == "__main__":
    move_flow:deque = selectMoveDataByScene("scene_4-1", True, ["tb3_0", "tb3_1", "tb3_2", "tb3_3"])

    i:int = 0
    while move_flow:
            
            goal_data = move_flow.popleft() 

            print("goal_data [%d] %s" % (i, goal_data))
            i += 1