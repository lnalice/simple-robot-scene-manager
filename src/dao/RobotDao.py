from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

cnx = connect_to_mysql(mysql_config, attempts=3)

"""
UPDATE a robot's status
- current module state: moduleState (4steps: 0, 0.25, 0.5, 0.75, 1)
- current location: displacementX, displacementZ
    (calculate the accumulated velocity value on the x-axis and z-axis)
"""
def updateRobotCurrentStatus (robotID, moduleState, displacementX, displacementZ) -> bool:
    updatedStatusInfo:list = [moduleState, displacementX, displacementZ, str(robotID)]

    cur = cnx.cursor(buffered=True)
    query = (
        "UPDATE Robot"
        "SET moduleState = %s, displacementX = %s, displacementZ = %s "
        "WHERE id = %s"
    )
    cur.execute(query, updatedStatusInfo)

    # cnx.close()

    return True

"""
GET a robot's status
"""
def moduleStateByRobotID(robotID: str) -> tuple:
    cur = cnx.cursor(buffered=True)

    query =  (
        "SELECT moduleState FROM Robot "
        "WHERE id = %s"
    )
    cur.execute(query, [robotID])

    statusInfo:tuple = cur.fetchone()

    # cnx.close()

    return statusInfo

def displacementByRobotID(robotID: str) -> tuple:
    cur = cnx.cursor(buffered=True)

    query =  (
        "SELECT displacementX, displacementZ FROM Robot "
        "WHERE id = %s"
    )
    cur.execute(query, [robotID])

    statusInfo:tuple = cur.fetchone()

    # cnx.close()

    return statusInfo