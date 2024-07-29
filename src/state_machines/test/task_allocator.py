import rospy
import smach
import smach_ros

class TaskAllocator:
    def __init__(self):
        self.tmp = 0


if __name__ == "__main__":
    simple_allocator = TaskAllocator()
    rospy.spin()