#!/usr/bin/env python

import roslib; roslib.load_manifest('project_rrt')
import rospy

from driver import driver

if __name__ == '__main__':
    try:
        pilot = driver()
        pilot.drive()
            
    except rospy.ROSInterruptException:
        pass
