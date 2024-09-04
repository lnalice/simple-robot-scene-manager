from collections import deque
from db.connection import connect_to_mysql
from db.config import mysql_config

cnx = connect_to_mysql(mysql_config, attempts=3)

def selectModuleDataByScene (scene: str, isOpposite: bool) -> deque:
    ctrl_flow = deque()

    result = cnx.cursor(buffered=True)
    query = (
        "SELECT robotID, degZ, degX, delay FROM SceneModule "
        "WHERE sceneID = %s"
    )
    result.execute(query, [scene])

    for (robotID, degZ, degX, delay) in result:
        task =""

        if degX == 0 and degZ == 0:
            continue

        if isOpposite:
            task = "%s %f %f %d" %(robotID, -float(degZ), -float(degX), delay)
        else:
            task = "%s %f %f %d" %(robotID, degZ, degX, delay)
        ctrl_flow.append(task)

    cnx.close()

    return ctrl_flow


if __name__ == "__main__":
    ctrl_flow:deque = selectModuleDataByScene("scene_1", False)

    i:int = 0
    while ctrl_flow:
            
            goal_data = ctrl_flow.popleft() 

            print("goal_data [%d] %s" % (i, goal_data))
            i += 1