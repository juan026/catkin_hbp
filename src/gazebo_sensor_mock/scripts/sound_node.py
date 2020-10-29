#!/usr/bin/env python
import rospy
from gazebo_msgs2.msg import SoundWithSource
from geometry_msgs.msg import Point
from std_msgs.msg import String
from math import floor
import random

sound_sources = [('bicycle', Point(-10, 10, 0)),
                      ('car', Point(5, -5, 0)),
                      ('cat', Point(3, -3, 0)),
                      ('dog', Point(10, 0, 0)) ]

def pub_sound():
    pub = rospy.Publisher('sounds', SoundWithSource, queue_size=10)
    rospy.init_node('gazebo_sensor_mock', anonymous=True)
    rate = rospy.Rate(1)
    n = 0 
    idx = floor(random.random() * 4)
    while not rospy.is_shutdown():
        n += 1
        if n >= 10:
            n = 0
            idx = floor(random.random() * 4)
        pub.publish(String(sound_sources[idx][0]), sound_sources[idx][1])
        rospy.loginfo('sending sound: {}'.format(sound_sources[idx][0]))
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_sound()
    except rospy.ROSInterruptException:
        pass