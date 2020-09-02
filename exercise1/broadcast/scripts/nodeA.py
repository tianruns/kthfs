#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('Sun',
                          Int16,
                          queue_size=1)
    rospy.init_node('nodeA',
                    anonymous=True)
    rate = rospy.Rate(20)  # 20hz
    k = 1
    while not rospy.is_shutdown():
        k = k + 4
        rospy.loginfo(k)
        pub.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

