#!/usr/bin/env python2

import rospy
import sense
import plan
import act

if __name__ == '__main__':
    rospy.init_node('command')

    # Steps
    # While ros is ok...
        # 1. Sense -- Get Sensor Values
        # sense = Sense()
        # 2. Plan -- Given sensor values, determine what to do
        # plan = Plan()
        # 3. Act -- Given a plan, act out the plan
        # Act.act(plan)
        # repeat until no plan is given
