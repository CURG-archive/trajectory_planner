#!/usr/bin/env python
import roslib
roslib.load_manifest( "trajectory_planner" )
import rospy
import trajectory_planner as tp
from HaoUtil import *
import tf_conversions.posemath as pm
from std_msgs.msg import String
import HaoUtil as hu
import pdb


class ShakeExecutor():
    """@brief - Shake the hand to test how stable the object is
    """
    def __init__( self, gd ):
        self.shake_listener = rospy.Subscriber("/graspit/shake", String, self.handle_shake)
        self.global_data = gd
        print 'shake executor initialized'
        
    def handle_shake(self, msg):
        print "start shaking..."
        jnts = hu.RotateHand(self.global_data,array([0,0,1]),'PALM',30*pi/180, True)
        jnts = hu.RotateHand(self.global_data,array([0,0,1]),'PALM',-60*pi/180, True)
        jnts = hu.RotateHand(self.global_data,array([0,0,1]),'PALM',60*pi/180, True)
        jnts = hu.RotateHand(self.global_data,array([0,0,1]),'PALM',-60*pi/180, True)
        jnts = hu.RotateHand(self.global_data,array([0,0,1]),'PALM',30*pi/180, True)


class AdjustExecutor():
    """@brief - Adjust the hand pose locally accordign to the new grasp stored in topic
    /graspit/adjustments.  Althou using /grasp_msgs.Grasp message, it is assumed the
    transform part is a adjustment transform in the current coordinate system of the
    hand.
    @member adjust_listener - subscriber to the graspit adjustment channel.  Expects graspit_msgs Grasp types
    """
    def __init__( self, gd ):
        self.adjust_listener = rospy.Subscriber("/graspit/adjustments", graspit_msgs.msg.Grasp, self.process_grasp_msg)
        #rospy.loginfo('Please make sure a OpenRave Environment is established already by initializing a trajectory_planner')
        self.global_data = gd
        print 'adjust executor initialized'
        
    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to make the adjustment specified by grasp_msg
        1. release the hand
        2. backup the hand
        3. plan a path to the a place 15cm back from the new pose
        4. use guarded motion to move forward
        """
        print 'adjustment:'
        print grasp_msg
        #release the object
        tp.open_barrett()

        #get the robot's current location and calculate the absolute tran after the adjustment
        #update the robot status
        tp.update_robot(self.global_data.or_env.GetRobots()[0])
        adjustInPalm = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
        arm_goal = tp.convert_cartesian_relative_goal(self.global_data, adjustInPalm)
        backup = numpy.matrix([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,-.05],
                               [0,0,0,1]]);
        back_from_arm_goal = arm_goal * backup

        #move back by 10cm
        #move_forward(-.1)
        #raw_input("Press enter to go to the target pose...")

        #update the robot status
        tp.update_robot(self.global_data.or_env.GetRobots()[0])
        #just go to the new place without a OpenRave trajectory plan
        #adjustInPalm = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
        #blocking motion set to be true
        #send_cartesian_relative_goal(self.global_data, adjustInPalm, True)
        send_cartesian_goal(back_from_arm_goal, True)
        raw_input("Press enter to use guarded motion to move forward...")

        #guarded move forward
        tp.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
        GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', 0.05, 20)
        raw_input("Press enter to close the hand...")

        #close the hand
        #tp.close_barrett()
        #tp.move_hand_velocity([0.5,0.5,0.5,0.0])
        GuardedCloseHand(self.global_data)
        
        selection = int(raw_input('Lift up (1) or not (0): '))
        if selection == 1:
            print 'lift up the object'
            success = tp.lift_arm(.1, True)
            if not success:
                grasp_status = graspit_msgs.GraspStatus.UNREACHABLE
                grasp_status_msg = "Couldn't lift object"
            else:
                print 'not lift up the object'


class MoveExecutor():
    """@brief - move the hand to the new pose according to topic
    /graspit/move and grasp from there.  Although using /grasp_msgs.Grasp message, it is assumed the
    transform part is a transform in the robot's base coordinate system.
    @member move_listener - subscriber to the graspit move channel.  Expects graspit_msgs Grasp types
    """
    def __init__( self, gd ):
        self.move_listener = rospy.Subscriber("/graspit/move", graspit_msgs.msg.Grasp, self.process_grasp_msg)
        self.global_data = gd
        print 'move executor initialized'
        
    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to make the adjustment specified by grasp_msg
        1. plan a path to the a place 15cm back from the new pose
        """
        print 'regular move:'
        print grasp_msg
        #release the object
        tp.open_barrett()

        #get the robot's current location and calculate the absolute tran after the adjustment
        #update the robot status
        tp.update_robot(self.global_data.or_env.GetRobots()[0])
        handGoalInBase = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))

        try:
            hand_tf = pm.toTf(pm.fromMatrix(handGoalInBase))
            bc = tf.TransformBroadcaster()
            now = rospy.Time.now()
            bc.sendTransform(hand_tf[0], hand_tf[1], now, "hand_goal", "armbase")
            self.global_data.listener.waitForTransform("armbase", "arm_goal", now, rospy.Duration(1.0))
            armtip_robot = self.global_data.listener.lookupTransform('armbase', 'arm_goal', now)
            armGoalInBase = pm.toMatrix(pm.fromTf(armtip_robot))
        except Exception, e:
            handle_fatal('convert_cartesian_world_goal failed: error %s.'%e) 

        backup_dist = 0.05#back five centimeters
        backup = numpy.matrix([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,-backup_dist],
                               [0,0,0,1]]);
        back_from_arm_goal = armGoalInBase * backup

        send_cartesian_goal(back_from_arm_goal, True)
        print 'regular move done'

        tp.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
        GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', backup_dist, 20)
        raw_input("Press enter to close the hand...")

        #close the hand
        GuardedCloseHand(self.global_data)

        
        selection = int(raw_input('Lift up (1) or not (0): '))
        if selection == 1:
            print 'lift up the object'
            success = tp.lift_arm(.1, True)
            if not success:
                grasp_status = graspit_msgs.GraspStatus.UNREACHABLE
                grasp_status_msg = "Couldn't lift object"
            else:
                print 'not lift up the object'


if __name__ == '__main__':
    try:
        rospy.init_node('adjustment_message_robot_server')
        rospy.loginfo('initializing')
        pdb.set_trace()
        ge = AdjustExecutor()
        se = ShakeExecutor()
        me = MoveExecutor()
        rospy.loginfo('done')
        loop = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            'foo'
            loop.sleep()
    except rospy.ROSInterruptException: pass

