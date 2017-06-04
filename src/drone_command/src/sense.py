#!/usr/bin/env python2
import rospy
import math
import tf2_ros as tf2
import geometry_msgs.msg

class Sense(object):
    """docstring for Sense."""
    def __init__(self):
        super(Sense, self).__init__()
        self.data = ()

    def get_camera_transformations(arg):
        tfBuffer = tf2.Buffer()
        listener = tf2.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        try:
            pass
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rate.sleep()
            continue
