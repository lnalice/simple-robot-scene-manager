#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from getkey import getkey, keys
import pandas as pd
import signal

def signal_handler(signum, frame):
    rospy.logwarn("Ctrl-c was pressed. Bye!")
    exit(1)

signal.signal(signal.SIGINT, signal_handler)

def newMsg(command: str, scene:str, names: str):
    command = command.upper().strip()
    scene_id = scene.strip()
    IDs = names.strip()
    
    msg = command + " " + scene_id + " " + IDs
    rospy.loginfo(f"New message is...'{msg}'", )

    return msg

if __name__ == "__main__":
    rospy.init_node("test_publisher")

    test_topic = '/react/commander'
    pub = rospy.Publisher(test_topic, String, queue_size=1)

    pointer = 0
    df = pd.read_csv('src/test/commands.csv')

    rospy.loginfo(f"\n{df.iloc[pointer]}")
    
    while not rospy.is_shutdown():
        key = getkey() 
        if key == keys.UP:
            pointer = max(0, pointer-1)
            rospy.loginfo(f"\n{df.iloc[pointer]}")

        elif key == keys.DOWN:
            pointer = min(len(df)-1, pointer+1)
            rospy.loginfo(f"\n{df.iloc[pointer]}")

        elif key == keys.ENTER:
            row = df.iloc[pointer]
            pub.publish(newMsg(row[0], row[1], row[2]))
            rospy.logwarn("I published now!")