#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray

import argparse

SECONDS= 10
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-c', '--command', required=False,
        help='command [MOVE, HOME, MODULE, SCENE]')
    parser.add_argument(
        '-n', '--names', required=False, nargs='+', help='Robot Names (multi-robot)') # ex) tb3_1 tb3_2
    parser.add_argument(
        '-an', '--name', required=False, help='Robot Name (multi-robot)') # ex) tb3_1 tb3_2
    parser.add_argument(
        '-s', '--scene', required=False, help='Scene ID')
    args = parser.parse_args()
    
    rospy.init_node("test_publisher")
    
    """
    /react/commander
    """
    test_topic = '/react/commander'
    pub = rospy.Publisher(test_topic, String, queue_size=1)

    command = args.command.upper()
    scene_id = args.scene
    IDs = ' '.join(args.names)
    
    msg = command + " " + scene_id + " " + IDs

    times = 0
    rospy.loginfo("I will send topic \"%s\"", msg)
    rospy.sleep(0.5)
    pub.publish(msg)
    
    rospy.loginfo("Done")
