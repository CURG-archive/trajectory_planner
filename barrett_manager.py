import roslib; roslib.load_manifest( "trajectory_planner" )
import rospy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs
import geometry_msgs
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg 
from tf import transformations as tr
import pr_msgs.msg
import pr_msgs.srv
from OWDUtil import *
from ft_manager import *


def move_hand(angles, blocking = True):
    """@brief - set joint angles of BarrettHand
       @param angles - target joint angles.
       @param blocking - if true, block until the hand stops moving

       Returns whether movement succeeded, why it failed if necessary, and the final position for the joint motions.
    """
    success = MoveHandSrv(1, angles)
    if not success:
        return success, "MovehandSrv Failed", []
    if blocking:
        success, reason, position = WaitForHand(0)
    else:
        reason = "nonblocking"
        position = []
    return success, reason, position

def move_hand_velocity(velocities, blocking = True):
    """@brief - set joint velocities of BarrettHand
       @param velocities - target joint velocities.
       @param blocking - if true, block until the hand stops moving

       Returns whether movement succeeded, why it failed if necessary, and the final position for the joint motions.
    """
    success = MoveHandSrv(2, velocities)
    if not success:
        return success, "MovehandSrv in velocitymode failed", []
    if blocking:
        success, reason, position = WaitForHand(0)
    else:
        reason = "nonblocking"
        position = []
    return success, reason, position

def move_hand_percentage(percentage):
    """@brief - set joint angles of BarrettHand relative to current position
       @param percentage - target relative positions.
    """
    jnts = get_barrett_joints()
    move_hand(array([jnts[0] * percentage, jnts[1] * percentage, jnts[2] * percentage, jnts[3]]))
    
def get_barrett_joints():
    """@brief - Get the current position of the hand.

       Returns the current joint position of the hand as a list.
    """
    msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
    return msg.positions
                           
def stop_barrett():
    """@brief - Set desired hand position to current hand position, stopping the hand

       Returns true of motion is successfully stopped.
    """
    try:
        msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
        MoveHandSrv(1,msg.positions)
        return 1, msg.positions
    except:
        return 0, []

def relax_barrett():
    """@brief - Stop applying torque to the hand.
    """
    return RelaxHandSrv()

def open_barrett(open_aperture = 0):
    """@brief - open the fingers without changing the spread angle
    """
    success = 0
    reason = "Exception in open barrett"
    positions = []
    try:
        msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
        success,reason,positions =  move_hand([open_aperture,open_aperture,open_aperture,msg.positions[3]])
        
    except:
        pass
    return success,reason, positions

def close_barrett():
    """@brief - move fingers completely closed in position mode.
    """
    return open_barrett(2)

def set_handstate_callback(global_data, callback_func):
    """@brief - Helper function to activate hand state callback
       @param global_data - data structure for storing handstate subscriber.
       @param callback_func - A function that process handstate messages. 
    """
    
    if not global_data.hs_sub == []:
        sub = global_data.hs_sub
        global_data.hs_sub = []
        sub.unregister()
        del sub
    global_data.hs_sub = rospy.Subscriber("/bhd/handstate", pr_msgs.msg.BHState, callback_func)

def unset_handstate_callback(global_data):
    """@brief - Helper function to deactivate hand state callback function
       @param global_data - data structure for storing handstate subscriber.
       
       Returns true if there is an active callback function, false otherwise. 
    """
    if not global_data.hs_sub == []:
        sub = global_data.hs_sub
        global_data.hs_sub = []
        sub.unregister()
        del sub
        return True
    return False


def guarded_close_single(active_fingers, desired_positions, tactile_threshold, ft_threshold, sleep_len = 5):
    """@brief - Close BarrettHand fingers until they reach a certain position or contact is made, one finger at a time.
       @param active_fingers - set which fingers are allowed to move.
       @param desired_positions - set desired positions for fingers.
       @param tactile_threshold - the reading for the tactile sensors which indicates contact
       @param ft_threshold - the reading for the force-torque sensor which indicates contact
       @param sleep_len - time to wait for completion of motion.

       Returns list of fingers that have not made contact and current position of fingers.
    """
    output_active_fingers = cp.deepcopy(active_fingers)
    for i in range(len(active_fingers)):
        my_active_fingers = [0,0,0]
        my_active_fingers[i] = active_fingers[i]
        ftm = FtFingerMonitor(my_active_fingers, desired_positions, tactile_threshold, ft_threshold)
        sleep(sleep_len)
        output_active_fingers[i] = ftm.active_fingers[i]
        ftm.close()
    return output_active_fingers, ftm.current_position

def guarded_stepwise_close(active_fingers, starting_positions, desired_positions, tactile_threshold, ft_threshold, num_steps = 10):
    """@brief - Close BarrettHand fingers incrementally until they reach a certain position or contact is made, one finger at a time.
       @param active_fingers - set which fingers are allowed to move.
       @param desired_positions - set desired positions for fingers.
       @param tactile_threshold - the reading for the tactile sensors which indicates contact
       @param ft_threshold - the reading for the force-torque sensor which indicates contact
       @param sleep_len - time to wait for completion of motion.

       Returns list of fingers that have not made contact and current position of fingers.
    """
    for i in range(num_steps):
        next_desired_positions = [((i+1.0)*(desired_positions[j]-starting_positions[j])/(float(num_steps)) +starting_positions[j]) for j in range(3)]
        next_desired_positions.append(desired_positions[3])
        print next_desired_positions
        active_fingers, current_position  = guarded_close_single(active_fingers, next_desired_positions, tactile_threshold, ft_threshold, .5)
        if not any(active_fingers):
            break
        
    return active_fingers, current_position

def get_tactile_threshold(multiplier = 5, offset = 1):
    def modify_data_tuple(t,m,o):
        return [r*m+o for r in t]
        
    tm = rospy.wait_for_message("/bhd/tactile", pr_msgs.msg.BHTactile, 5)
    tm.finger1 = modify_data_tuple(tm.finger1,multiplier, offset)
    tm.finger2 = modify_data_tuple(tm.finger2,multiplier, offset)
    tm.finger3 = modify_data_tuple(tm.finger3,multiplier, offset)
    tm.palm = modify_data_tuple(tm.palm,multiplier, offset)
    return tm



class FingerMotionMonitor():
    """@brief - A monitor that is activated by the state of the barrett hand. It monitors both the joint state and
                the tactile sensors of the hand.
    """
    def __init__(self, active_fingers, desired_position, tactile_threshold):
        """@brief - Create a new monitor

           @param active_fingers - Which fingers are active right now
           @param desired_position - The hand state that the monitor is waiting to achieve.
           @param tactile_threshold - The limit at which the tactile sensors are considered activaed.
        """
        self.active_fingers = active_fingers
        self.desired_positions = desired_position
        self.current_position = []
        self.current_strain = []
        self.current_tactile = []
        #Parameters for linear model of strain that depends on finger position.
        self.strain_model = [3000.0,0]
        
        hs_msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState)
        #initial call of hand state functor. Necessary to intialize current strain and position.
        self.hand_state_func(hs_msg)
        self.tactile_threshold = tactile_threshold      
        self.handstate_subscriber = rospy.Subscriber("/bhd/handstate", pr_msgs.msg.BHState, self.hand_state_func)        
        self.tactile_subscriber = rospy.Subscriber("/bhd/tactile", pr_msgs.msg.BHTactile, self.tactile_state_func)
        self.reset_desired_position()

        
    def reset_desired_position(self):
        """@brief - Set the desired positions for the hand to the current position

        """
        desired_positions = list(self.current_position)
        desired_positions[-1] = self.desired_positions[-1]
        for i in range(len(self.active_fingers)):
            if self.active_fingers[i] == 1:
                desired_positions[i] = self.desired_positions[i]
        self.desired_positions = desired_positions
        print "ft_finger_move desired positions:"
        print self.desired_positions
        MoveHandSrv(1,self.desired_positions)

    def hand_state_func(self,  hs_msg):
        """@brief - The callback function for the handstate listener.

        Reads the current state of hand positions, strains, and whether the state of the
        hand has gone beyond some of the set thresholds.

           @param hs_msg - The input handstate message.
        """
        self.current_position = hs_msg.positions
        self.current_strain = hs_msg.strain
        any_change = False
        for i in range(len(self.current_strain)):
            if not self.active_fingers[i]:
                continue
            if (self.current_strain[i] > self.strain_model[1] * self.current_position[i]
                                                                 + self.strain_model[0]):
                print "high strain on finger %i: %d"%(i, self.current_strain[i])
                
                self.active_fingers[i] = 0
                any_change = True

        if any_change:
            self.reset_desired_position()

    def tactile_state_func(self, tac_msg):
        """@brief - The callback function for the tactile sensors of the hand.

           @param tac_msg - The input tactile message.
        """
        def any_greater(l1, l2):
             return [i for i in range(len(l1)) if l1[i] > l2[i]] != []

        #If any of the taxels are above the predetermined threshold, set the finger to inactive.
        any_change = False
        if any_greater(tac_msg.finger1, self.tactile_threshold.finger1):
            print "high tactile 1"
            any_change = True
            self.active_fingers[0] = 0
            
        if any_greater(tac_msg.finger2, self.tactile_threshold.finger2):
            print "high tactile 2"
            any_change = True
            self.active_fingers[1] = 0

        if any_greater(tac_msg.finger3, self.tactile_threshold.finger3):
            print "high tactile 3"
            any_change = True
            self.active_fingers[2] = 0

        if any_change:
            self.reset_desired_position()

    def close(self):
        """@brief - Unsubscribe function that is part of the destructor for this function.
        """
        try:
            self.handstate_subscriber.unregister()
            self.tactile_subscriber.unregister()
        except:
            print "unregister failed"


class FtFingerMonitor(FingerMotionMonitor):
    """@brief - FingerMotionMonitor with added capabilities to monitor the force torque sensor as well.
    """
    def __init__(self, active_fingers, desired_position, tactile_threshold, ft_threshold):
        """@brief - Create new FtFingerMonitor

           @param active_fingers - The fingers that are allowed to move
           @param desired_position - The goal pose of the hand.
           @param tactile_threshold - The threshold for tactile sensors to inactivate their respective finger.
           @param ft_threshold - The threshold for the force-torque sensor force level to deactivate the hand.        
        """
        self.ft_sub = []
        self.handstate_subscriber = []
        self.tactile_subscriber = []
        self.active_fingers = []
        self.desired_positions = []
        self.current_position = []
        self.current_strain = []
        self.current_tactile = []
        self.strain_model = [3000.0]

        tare_FT()
        FingerMotionMonitor.__init__(self,active_fingers, desired_position, tactile_threshold)
        self.ft_sub = rospy.Subscriber("ForceTorque/Readings", geometry_msgs.msg.Wrench, self.ft_callback)
        self.hits = 5
        self.delay_num = 5
        self.ft_threshold = ft_threshold

    def ft_callback(self, wrench_msg):
        """@brief - Test if the total force is above some threshold.

        Because the sensor is noisy, we make sure that the threshold is crossed for a certain number of sensor
        readings.
        """
        if linalg.norm(array([wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z])) > self.ft_threshold:
            self.hits -= 1
            print self.hits
        else:
            self.hits = self.delay_num
        if self.hits <= 0:
            self.active_fingers = [0,0,0]
            self.reset_desired_position()

    def reset_desired_position(self):
        """@brief - Test if all fingers have been inactivated. If it has, stop the monitor.
        """
        FingerMotionMonitor.reset_desired_position(self)
        if not any(self.active_fingers):
            self.close()
        
    def close(self):
        """@brief - Unregister all active subscribers before destroying the monitor.
        """
        print 'closing'
        try:
            self.ft_sub.unregister()
        except:
            print "ft_already_closed"
        
        FingerMotionMonitor.close(self)
