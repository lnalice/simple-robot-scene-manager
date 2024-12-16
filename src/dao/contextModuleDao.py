from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

import rospy

def selectModuleDataByContextID (contextID: str) -> deque:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)
    
    print(f"contextID is {contextID}.")
    query = (
        "SELECT role, roleID, state, delay, `order` FROM ContextModule "
        "WHERE contextID = %s "
        "ORDER BY `order` ASC;"  
    )
    cur.execute(query, [contextID])
    
    result = cur.fetchall()

    cnx.close()

    rospy.loginfo(f"result {result}")

    return result