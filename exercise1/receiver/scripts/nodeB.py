#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

data = None


def callback(msg):
    global data
    data = msg.data / 0.15


if __name__ == '__main__':
    rospy.init_node('nodeB',
                   anonymous=True)
    sub = rospy.Subscriber('Sun',
                     Int16,
                     callback)
    pub = rospy.Publisher('/kthfs/result',
                          Int16,
                          queue_size=1)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        pub.publish(data)
        rospy.loginfo(data)
        rate.sleep()










