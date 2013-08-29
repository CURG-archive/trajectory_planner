import roslib 
roslib.load_manifest("owd")

from pr_msgs.srv import Reset
from pr_msgs.srv import SetSpeed
from pr_msgs.srv import MoveHand
from pr_msgs.srv import GuardedMove
from pr_msgs.srv import SetStiffness
from pr_msgs.srv import ResetFinger
from pr_msgs.srv import RelaxHand
import pr_msgs.msg
import time
from numpy import *
import rospy

def MoveHandSrv(movetype, movevalues):
    try:
        owd_movehand = rospy.ServiceProxy('/bhd/MoveHand', MoveHand)
        res = owd_movehand(movetype, movevalues)
        #print res
        return 1
    except rospy.ServiceException, e:
        print "Service call to MoveHand failed: %s"%e
        return 0

def RelaxHandSrv():
    try:
        owd_relaxhand = rospy.ServiceProxy('/bhd/RelaxHand', RelaxHand)
        res = owd_relaxhand()
        return 1
    except rospy.ServiceException, e:
        print "Service call to MoveHand failed: %s"%e
        return 0
'''
This function will update hand pose 
'''

def GetHandPose( whichmanip ):
    try:
        msg = rospy.wait_for_message(topic = "/bhd/handstate", topic_type
                = pr_msgs.msg.BHState, timeout = 0.5 ) 
    except rospy.ROSException as ex:
        rospy.logwarn( 'Cannot get BHState: %s' % ex )
        return 0, ex, array( [0, 0, 0, 0] )
    
    return 1, '', array( msg.positions )


def HandVelocityClose( velocity = 0.1 ):
    velocity = max( 0.05, min( 1.25, velocity ) )
    return MoveHandSrv( 2,( velocity, velocity, velocity, 0))

def HandVelocityOpen( velocity = 0.1 ):
    velocity = -max( 0.05, min( 1.25, velocity ) )
    return MoveHandSrv( 2,( velocity, velocity, velocity, 0))

def HandPositionOpenMaximalDistention():
    return MoveHandSrv(1,( 1.1, 1.1, 1.1, 0))

'''
PositionMove to the specified angles 
angles could be of length 1 or four
four format: f1, f2, f3, spread
ignoreFingers only used when the angles contain
4 numbers, if ignorefingers is set to True,
the numbers passed in for the fingers are not used
only the spread angle is used in this case
'''
def HandPositionMove( angles , ignoreFingers = False):
    angles = array( [angles] ).flatten()
    if len(angles) == 1:
        return MoveHandSrv(1,( angles[0], angles[0], angles[0], 0))
    elif len(angles) == 4:
        if(ignoreFingers):
            return MoveHandSrv(1,( pi, pi, pi, angles[3]))
        return MoveHandSrv(1,( angles[0], angles[1], angles[2], angles[3]))
    return 0

'''
fully open the hand or
keep the spread angle not moved (paritally = True with global_data passed in) or
keep the spread angle not moved and open only a small amount (partially = True, global_data passed in, and openToExtent specifies the how much of the hand should be opened from the current joint angles

'''
def HandPositionOpen(partially = False, global_data = None, openToExtent = 0):
    if(not partially):
        return MoveHandSrv(1,( 0, 0, 0, 0))
    else:
        if(global_data == None):
            rospy.logerr( 'global_data not passed in while using HandPositionOpen' )
            return
        dofs = global_data.orEnv.GetRobot('BarrettWAM').GetDOFValues();
        return MoveHandSrv(1,(dofs[7] * openToExtent, dofs[8] * openToExtent, dofs[9] * openToExtent, dofs[10]))




def HandOpenFeedback( whichmanip ):
    '''Blocking call to HandPositionOpen that returns after a timeout or after
    finishing movement
    '''
    HandPositionOpen()
    #sleep for a moment to make sure movement has time to start
    time.sleep(.1)
    success, reason, positions = WaitForHand( whichmanip )
    
    success, reason = IsHandOpened( positions )
    if not success:
        rospy.logerr( 'HandOpen::Unable to open hand: %s' % reason )
    return success, reason


def HandPositionClose(spread_angle = 0):
    angle = pi
    return MoveHandSrv(1,( angle, angle, angle, spread_angle))


def WaitForHand( whichmanip, timeout=5.0 ):
    '''
    Get hand pose and return as soon as it stops moving (or after timeout)
    '''
    waittime = 0.25
    numiter = int(floor(timeout / waittime))
    success, reason, positions = GetHandPose( whichmanip )
    for i in range( numiter ):
      lastpos = positions
      time.sleep( waittime )
      success, reason, positions = GetHandPose( whichmanip )
      rospy.loginfo( '  hand pose: %s' % ( array( positions ) ) )
      if sum( abs( lastpos - positions ) ) < 0.01:
          rospy.loginfo( 'break' )
          break

    return success, reason, positions


def ResetHand( global_data, whichmanip ):
    '''
    To be updated to do the following. 
    Resets hands and breakaway using OWD call
    '''
    success, reason = ResetFingers( [1,2,3])
    SetHandSpeedScalar( __MAX_HAND_SPEED )

    return success, reason

def CheckHand( global_data, whichmanip ):
    '''
    To be updated to do the following. Assumes fingers are open.
    1. Closes hand and checks if hands are closed 
    2. Opens hand and checks if hands are open
    The new function will use blocking calls to OWD
    '''
    # Fully close
    HandPositionMove(pi)
    success, reason, positions = WaitForHand( whichmanip )
    time.sleep( 2 )
    success, reason = IsHandClosed( positions )
    if not success:
        rospy.logwarn( 'Unable to close hand: %s' % reason )
        return success, reason

    rospy.loginfo( 'Successfully closed hand' )

    # Fully open
    HandPositionOpen( True, global_data )
    success, reason, positions = WaitForHand( whichmanip )
    HandPositionMove( 0 )
    success, reason, positions = WaitForHand( whichmanip )

    success, reason = IsHandOpened( positions )
    if not success:
        rospy.logwarn( 'Unable to open hand: %s' % reason )
        return success, reason

    rospy.loginfo( 'Successfully opened hand' )

    return 1, '' 
'''
Functions to set hand speed 
'''
__UNIT_HAND_SPEED_ARRAY = array([1., 1., 1., 1.])
__MIN_HAND_SPEED = 0.2
__MAX_HAND_SPEED = 1.5
__MAX_HAND_ACCEL = 0.2


def SetHandSpeed( jointSpeedArray, jointAccelerationLimit ):
    success = 0
    rospy.loginfo( "Hand speed: %s " % jointSpeedArray )
    try:
        success = 1
        SetSpeedSrv = rospy.ServiceProxy( '/bhd/SetSpeed', SetSpeed )
        res = SetSpeedSrv( jointSpeedArray, jointAccelerationLimit )
    except rospy.ServiceException, reason:
        rospy.logerr( "Service call to SetSpeed failed: %s" % reason )
    return success


def SetHandSpeedScalar( speed ):
    if speed > __MAX_HAND_SPEED:
        speed = __MAX_HAND_SPEED
    elif speed < __MIN_HAND_SPEED:
        speed = __MIN_HAND_SPEED

    handSpeed = speed * __UNIT_HAND_SPEED_ARRAY
    return SetHandSpeed( handSpeed, __MAX_HAND_ACCEL );
