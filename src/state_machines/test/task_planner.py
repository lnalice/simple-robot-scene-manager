import rospy
import smach
import smach_ros

class TaskPlanner:
    def __init__(self):
        self.tmp = 0


if __name__ == "__main__":
    simple_planner = TaskPlanner()
    rospy.spin()