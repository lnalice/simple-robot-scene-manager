from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

import rospy

"""
GET module's degree data by state
- degree data: degZ, degX (확장 정도 별 실제 모터 각도 변위값)
- state: 0, 0.25, 0.5, 0.75, 1 (확장 정도)
"""
def selectDegreeByState(state: float) -> tuple:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)
    
    rospy.logwarn(state)
    query = (
        "SELECT degZ, degX FROM Module "
        "WHERE state = %s"
    )
    cur.execute(query, [state])

    degreeInfo:tuple = cur.fetchone()

    cnx.close()
    
    return degreeInfo # tuple(degZ, degX)

def selectStateByDegree(degZ: float, degX: float) -> tuple:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)
    
    query = (
        "SELECT state FROM Module "
        "WHERE degZ = %s"
    )
    cur.execute(query, [degZ])

    degreeInfo:tuple = cur.fetchone()

    cnx.close()
    
    return degreeInfo # tuple(state)