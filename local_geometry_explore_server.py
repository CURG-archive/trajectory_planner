#!/usr/bin/env python
import roslib
roslib.load_manifest( "trajectory_planner" )
import rospy
import trajectory_planner as tp
from HaoUtil import *
import tf_conversions.posemath as pm
from std_msgs.msg import String
from graspit_msgs.msg import Grasp, Adjust
from geometry_msgs.msg import Pose
from graspit_ros_server.srv import StartExplore, EndExplore, AddPointCloud
import HaoUtil as hu
import pdb
import copy

class LocalGeometryExplore():
    """@brief - Locally explore the surface of the object and collect point clouds
    """
    def __init__( self, gd ):
        self.adjust_listener = rospy.Subscriber("/graspit/explore", String, self.process_explore)
        rospy.loginfo('Please make sure a OpenRave Environment is established already by initializing a trajectory_planner')
        self.global_data = gd
        rospy.loginfo('waiting for services to be advertised')
        rospy.wait_for_service('start_explore')
        rospy.wait_for_service('end_explore')
        rospy.wait_for_service('add_point_cloud')
        rospy.loginfo('everything is ready to go')
        
    def process_explore(self, explore_msg):
        """@brief - Attempt to explore
        """
        #number of points we want to collect along x, z directions
        count_x = 5
        count_z = 5
        #distance we want to cover along x, z directions
        dist_x = 0.03
        dist_z = 0.03
        tp.open_barrett()
        #first want to find the boundary along z-direction, for 3cm to test
        distance_moved, isHit = hu.GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', dist_z / 2)
        #pdb.set_trace()
        #move back to the ideal location to start local exploration
        hu.GuardedMoveUntilHit(self.global_data, array([0,0,-1]), 'PALM', dist_z, 500)

        #start local exploration
        count = 0
        self.start_explore()
        adjust_msg = Adjust()#identity matrix initially
        adjust_z_msg = Adjust()
        adjust_accumulated = Adjust()
        adjust_msg.offset.orientation.w = 1#it is zero initially
        adjust_msg.offset.position.x = dist_x / (count_x-1) #step size:3mm along x
        adjust_z_msg.offset.orientation.w = 1
        adjust_z_msg.offset.position.z = dist_z / (count_z-1) #step size:5mm along z
        adjust_accumulated.offset.orientation.w = 1

        #initial x,z is set so that the fingers are centered at the middle point of the range
        adjust_accumulated.offset.position.x =  dist_x / 2.0
        adjust_accumulated.offset.position.z = - dist_z / 2.0
        
        """
        At the beginning we need to make sure the hand is backed up
        This is the first step for local reconstruction
        """
        pdb.set_trace()
        self.adjust(adjust_accumulated)
        #hu.GuardedCloseHand(self.global_data)
        #tp.move_hand_velocity([0.1,0.1,0.1,0])
                
        while (count < count_x * count_z):
            #pdb.set_trace()
            self.collect_point_cloud(adjust_accumulated.offset)
            if(count == count_x * count_z - 1):
                break
            #make a move
            if mod(count+1, count_x) == 0: #end of one vertical line, move forward
                self.adjust(adjust_z_msg)
                adjust_accumulated.offset.position.z = adjust_accumulated.offset.position.z + adjust_z_msg.offset.position.z
            else:#not the end of the vertical line
                if mod(count, count_x) == 0: #start of one vertical line, change up/down direction and move up/down
                    adjust_msg.offset.position.x = - adjust_msg.offset.position.x
                adjust_accumulated.offset.position.x = adjust_accumulated.offset.position.x + adjust_msg.offset.position.x
                self.adjust(adjust_msg)
            count = count + 1
        #now go back to the original position
        adjust_back = Adjust()
        adjust_back.offset.position.x = - adjust_accumulated.offset.position.x
        adjust_back.offset.position.z = - adjust_accumulated.offset.position.z
        pdb.set_trace()
        self.adjust(adjust_back, False)
        self.end_explore()
        
    def collect_point_cloud(self, offset):
        #collect the current hand joint values and tactile sensor values
        hsmsg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
        htmsg = rospy.wait_for_message("/bhd/tactile", pr_msgs.msg.BHTactile, 5)
        try:
            add_point_cloud = rospy.ServiceProxy('add_point_cloud', AddPointCloud)
            offset_in_mm = copy.deepcopy(offset)
            offset_in_mm.position.x = offset.position.x * 1000
            offset_in_mm.position.y = offset.position.y * 1000
            offset_in_mm.position.z = offset.position.z * 1000
            response = add_point_cloud(hsmsg, htmsg, offset_in_mm)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        
    def start_explore(self):
        print "start"
        try:
            start_explore_service = rospy.ServiceProxy('start_explore', StartExplore)
            response = start_explore_service()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def end_explore(self):
        print "finish"
        try:
            end_explore_service = rospy.ServiceProxy('end_explore', EndExplore)
            response = end_explore_service()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def adjust(self, adjust_msg, close_hand = True):
        """
        adjust_msg specifies the adjustment in palm's local coordinate system
        """
        print 'adjust:'
        print adjust_msg
        #release the object
        tp.open_barrett()
        #tp.move_hand_percentage(0.8)

        #get the robot's current location and calculate the absolute tran after the adjustment
        #update the robot status
        tp.update_robot(self.global_data.or_env.GetRobots()[0])
        adjustInPalm = pm.toMatrix(pm.fromMsg(adjust_msg.offset))
        arm_goal = tp.convert_cartesian_relative_goal(self.global_data, adjustInPalm)
        backup = numpy.matrix([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,-.05],
                               [0,0,0,1]]);
        back_from_arm_goal = arm_goal * backup

        #update the robot status
        tp.update_robot(self.global_data.or_env.GetRobots()[0])
        #just go to the new place without a OpenRave trajectory plan
        #adjustInPalm = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
        #blocking motion set to be true
        #send_cartesian_relative_goal(self.global_data, adjustInPalm, True)
        #send_cartesian_goal(back_from_arm_goal, True)
        send_cartesian_goal(arm_goal, True)
        #raw_input("Press enter to use guarded motion to move forward...")

        #guarded move forward
        #GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', 0.05, 20)
        #raw_input("Press enter to close the hand...")

        #close the hand
        #tp.close_barrett()

        #gentlly close the fingers
        if close_hand:
            hu.GuardedCloseHand(self.global_data)
            tp.move_hand_velocity([0.1,0.1,0.1,0])
        
"""        
if __name__ == '__main__':
    try:
        rospy.init_node('local_geometry_explore_server')
        pdb.set_trace()
        #ge = AdjustExecutor()
        #se = ShakeExecutor()
        #lge = LocalGeometryExplore()
        rospy.loginfo('done')
        loop = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            'foo'
            loop.sleep()
    except rospy.ROSInterruptException: pass

"""
