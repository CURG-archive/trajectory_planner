#!/usr/bin/env python
from trajectory_planner import *
import IPython

if __name__ == '__main__':
    rospy.init_node('manual_control')
    use_real_hw = rospy.get_param('use_real_hw')
    g = SetupStaubliEnv(False, use_real_hw)

    IPython.embed()
