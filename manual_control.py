#!/usr/bin/env python
from trajectory_planner import *
import IPython

if __name__ == '__main__':
    rospy.init_node('manual_control')
    init_planner = rospy.get_param('init_planner')
    g = SetupStaubliEnv(False, init_planner)

    IPython.embed()
