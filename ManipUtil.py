# Copyright (c) 2010-2011 REARM Team 
# 
#   Author: Aravind Sundaresan <aravind@ai.sri.com>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import pdb
import rearm
from numpy import pi, mat, array, arange, fabs, eye, dot

import roslib 
roslib.load_manifest("sri_manip_actions")
import rospy

from pr_msgs.srv import Reset
from pr_msgs.srv import SetSpeed
from pr_msgs.srv import MoveHand
from pr_msgs.srv import GuardedMove
from pr_msgs.srv import SetStiffness
from pr_msgs.srv import ResetFinger
from gfe_owd_plugin.srv import AddWSTraj
from pr_msgs.msg import Joints
from openravepy import *

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
#import rearm_msgs.msg


from GraspUtil import *
from GraspVerifyUtil import *
from TrajectoryUtil import *
from OWDUtil import *
import OpenRaveUtil

import TSRUtil
import TSRPose
from manipapp.manipactions.baseaction import *
import manipapp.util.TSR as TSR
import subprocess 
import primitive_prototypes
import primitive_prototypes.srv
import ActionMonitor
# set logger levels for this function 
logdebug = rospy.logdebug
loginfo = rospy.loginfo
logwarn = rospy.logwarn
logerr = rospy.logerr

# Taken from libcd's kin.c
# 2011-08-01 cdellin
def cd_kin_H_from_op_diff(pos_from, pos_to_diff):
   '''
   Produce a transform H rooted at location pos_from
   with Z axis pointed in direction pos_to_diff
   '''
   H = eye(4)
   # Set d
   H[0,3] = pos_from[0]
   H[1,3] = pos_from[1]
   H[2,3] = pos_from[2]
   # Define Z axis in direction of arrow */
   len = sqrt(dot(pos_to_diff,pos_to_diff))
   H[0,2] = pos_to_diff[0]/len
   H[1,2] = pos_to_diff[1]/len
   H[2,2] = pos_to_diff[2]/len
   # Define other axes
   if fabs(H[0,2]) > 0.9:
      # Z is too close to e1, but sufficiently far from e2
      # cross e2 with Z to get X (and normalize)
      len = sqrt(H[2,2]*H[2,2] + H[0,2]*H[0,2])
      H[0][0] = H[2,2] / len
      H[1][0] = 0.0
      H[2][0] = -H[0,2] / len
      # Then Y = Z x X
      H[0,1] = H[1,2] * H[2,0] - H[2,2] * H[1,0]
      H[1,1] = H[2,2] * H[0,0] - H[0,2] * H[2,0]
      H[2,1] = H[0,2] * H[1,0] - H[1,2] * H[0,0]
   else:
      # Z is sufficiently far from e1;
      # cross Z with e1 to get Y (and normalize)
      len = sqrt(H[2,2]*H[2,2] + H[1,2]*H[1,2])
      H[0,1] = 0.0
      H[1,1] = H[2,2] / len
      H[2,1] = -H[1,2] / len
      # Then X = Y x Z
      H[0,0] = H[1,1] * H[2,2] - H[2,1] * H[1,2]
      H[1,0] = H[2,1] * H[0,2] - H[0,1] * H[2,2]
      H[2,0] = H[0,1] * H[1,2] - H[1,1] * H[0,2]
   return H

def invert_H(H):
   '''
   Invert transform H
   '''
   R = H[0:3,0:3]
   d = H[0:3,3]
   Hinv = eye(4)
   Hinv[0:3,0:3] = R.T
   Hinv[0:3,3] = -dot(R.T, d)
   return Hinv

'''
Rotational transform from world to wam0 coordinates
'''
__R0_wam0 = array( [ [0, 0, sqrt(2)], [-1, -1, 0], [1, -1, 0] ] ) / sqrt( 2 )

def WorldToWam0( forcedir ):
    '''
    Function to transform from world coordinates to WAM0 coordinates 
    To be used with ApplyForce whose input is in WAM0 coordinates
    forcedir must contain three elements
    '''
    return dot( __R0_wam0, array( forcedir ).flatten() )


class MoveType:
    """
    Move type determines the method to move to final 
    """
    to_contact, to_final = range( 2 )
    
    @staticmethod 
    def toStr( movetype ):
        if movetype == MoveType.to_contact:
            return "to_contact"
        elif movetype == MoveType.to_final:
            return "to_final"
        return "unknown"



def PersistentUpdateRobot(global_data, whichmanip):
    '''
    Deprecated: Use 
    success, reason = OpenRaveUtil.UpdateRobot2( global_data ) instead
    '''
    robsuccess, robstatus = UpdateRobot( global_data, whichmanip )
    while not robsuccess:
        if StatusStopEverything(robstatus):
            rospy.logwarn("Action should be quiting - stopeverything was called")
            return robsuccess, robstatus
        robsuccess, robstatus = UpdateRobot( global_data, whichmanip )
        time.sleep( 0.2 )
    return True, ''


def UpdateRobot( global_data, whichmanip ):
    '''
    Deprecated: Use 
    success, reason = OpenRaveUtil.UpdateRobot2( global_data ) instead
    '''
    global_data.orEnv.LockPhysics(False) 
    success, status = WaitForRobot(global_data)
    if not success:
        
        global_data.orEnv.LockPhysics(True) 
        return 0,"Robot not done. %s"%(status)       
    global_data.orEnv.LockPhysics(True)

    dofvalues = global_data.robot.GetDOFValues()  
    
    dofindices = global_data.armdofs[whichmanip];
    dofstr = ' '.join( [ `"%.3f" % dofvalues[i]` for i in dofindices] )
    dofindices = global_data.handdofs[whichmanip];
    dofstr = dofstr + ':' + ' '.join( [ `"%.3f" % dofvalues[i]` for i in dofindices] )
    rospy.logdebug( "angles: %s" % dofstr )
    
    return 1, "Done"

'''
This function will update hand pose 
'''

def GetHandPose( whichmanip ):
    try:
        msg = rospy.wait_for_message(topic = "/right/bhd/handstate", topic_type
                = pr_msgs.msg.BHState, timeout = 0.5 ) 
    except rospy.ROSException as ex:
        rospy.logwarn( 'Cannot get BHState: %s' % ex )
        return 0, ex, array( [0, 0, 0, 0] )
    
    return 1, '', array( msg.positions )


def GetFullHandPose( whichmanip ):
    """
    Includes secondary encoders 
    """
    try:
        msg = rospy.wait_for_message(topic = "/right/bhd/handstate", topic_type
                = pr_msgs.msg.BHState, timeout = 0.5 ) 
    except rospy.ROSException as ex:
        rospy.logwarn( 'Cannot get BHState: %s' % ex )
        return 0, ex, array( [0, 0, 0, 0] )
    positions = list(msg.positions)
    positions.extend(msg.secondary_positions)
    return 1, '', array([positions ])

'''
'''
def IsHandClosed( positions ):
    failed = [i for i in range(3) if positions[i] < 2.25 ]
    if len( failed ) > 0:
        reason = '%d fingers %s failed' % ( len( failed ), failed )
        return 0, reason
    return 1, ''


'''
'''
def IsHandOpened( positions ):
    failed = [i for i in range(3) if abs( positions[i] ) > 0.1]
    if len( failed ) > 0:
        reason = '%d fingers %s failed' % ( len( failed ), failed )
        return 0, reason
    return 1, ''


def WaitForHand( whichmanip, timeout=5.0 ):
    '''
    Get hand pose and return as soon as it stops moving (or after timeout)
    '''
    waittime = 0.25
    numiter = timeout / waittime
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
Tare force-torque sensor
'''
def TareForceTorque():
    ftt = rospy.ServiceProxy('/right/owd/ft_tare', Reset)
    ftt()
    return


def ReadForceTorque():
    '''Wait for one force-torque message,
    unpack it and return it
    '''

    try:
        msg = rospy.wait_for_message("/right/owd/forcetorque", topic_type= geometry_msgs.msg.WrenchStamped)
    except:
        rospy.logwarn("Failed to recieve force torque sensor message")
        return 0, "Failed to recieve force torque sensor message", []
    return 1, "success", array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])


#TODO: TO BE TESTED
'''
Reset the fingers of the hand so that we can clear the breakaway
This is a blocking function
'''
def ResetFingers(fingers = [1,2,3]):
    success = 1
    reason = "Reset failed for fingers: "
    for f in fingers:
        try:
            ResetFingerSrv = rospy.ServiceProxy('right/bhd/ResetFinger', ResetFinger)
            res = ResetFingerSrv(f)
            if not res.ok:
                success = 0
                reason = reason + ( "%d " % f )
                rospy.logerr( "reset finger %s failed" % f )
        except rospy.ServiceException, reason:
            rospy.logerr( "Service call to ResetFinger failed: %s" % reason )

    if success == 1:
        reason = "Fingers successfully reset"
    return success, reason


'''
Functions to set arm speed 
'''
__MAX_ARM_SPEED = array([1., 1., 1., 1., 2., 2., 1.])
__MAX_ARM_ACCEL = 0.2

def SetArmSpeed( jointSpeedArray, jointAccelerationLimit ):
    success = 0
    try:
        success = 1
        SetSpeedSrv = rospy.ServiceProxy('/right/owd/SetSpeed',SetSpeed)
        res = SetSpeedSrv(jointSpeedArray, jointAccelerationLimit)
    except rospy.ServiceException, reason:
        rospy.logerr( "Service call to SetSpeed failed: %s" % reason )
    return success

def SetArmSpeedSuperSlow():
    return SetArmSpeed( __MAX_ARM_SPEED * 0.075, __MAX_ARM_ACCEL * 0.05 );

def SetArmSpeedSlow():
    return SetArmSpeed( __MAX_ARM_SPEED * 0.15, __MAX_ARM_ACCEL * 0.5 );
     
def SetArmSpeedFast():
    return SetArmSpeed( __MAX_ARM_SPEED * 0.95, __MAX_ARM_ACCEL * 1.0 );

def SetArmSpeedRate( rate , accel_rate = 1.0):
    # rate should be between 0.05 and 1
    if rate > 1:
        rate = 0.99
    elif rate < 0.05:
        rate = 0.05

    if accel_rate > 1:
        accel_rate = 1.0
    elif accel_rate < 0.05:
        accel_rate = 0.05

    return SetArmSpeed( __MAX_ARM_SPEED * rate, __MAX_ARM_ACCEL *accel_rate);

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
        SetSpeedSrv = rospy.ServiceProxy( '/right/bhd/SetSpeed', SetSpeed )
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


'''
Other services 
'''
def SetCustomStiffness( stiff ):
    try:
        SetStiffnessSrv = rospy.ServiceProxy( '/right/owd/SetStiffness', SetStiffness )
        res = SetStiffnessSrv( stiff )
    except rospy.ServiceException, reason:
        rospy.logerr( "Service call to SetStiffness failed: %s" % reason )
    return 1

'''
Move hand service 
'''
def MoveHandSrv(movetype, movevalues):
    try:
        owd_movehand = rospy.ServiceProxy('/right/bhd/MoveHand', MoveHand)
        res = owd_movehand(movetype, movevalues)
        return 1
    except rospy.ServiceException, e:
        print "Service call to MoveHand failed: %s"%e
        return 0

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
    angles = numpy.array( [angles] ).flatten()
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


def HandPositionClose():
    angle = pi
    return MoveHandSrv(1,( angle, angle, angle, 0))

'''
Start bag file with list of topics
'''
def StartBagFile(topiclist, output_prefix):
     topiclist.insert(0,"rosbag")
     topiclist.insert(1,"record")
     topiclist.insert(2,"--output-prefix=" + output_prefix)
     return subprocess.Popen(topiclist)

'''
End bag file: will end all 'rosbag record' functions (?) 
'''
def EndBagFile(bagProcess):
     return bagProcess.send_signal(subprocess.signal.SIGINT)
        
'''
Attach specified object to the robot 
If no object is specified, look for objects in collision with hand 
Also SetExtra mass for link 7 for specific objects
Returns success, followed by reason  
'''
def AttachObject( global_data, whichmanip, targetName = "" ):
    orEnv = global_data.orEnv
    robot = global_data.robot
    
    # set contact object name to target object name
    contactObjName = targetName.split()[0]
    if not bool( contactObjName ):
        # if targetName is not specified, *then* look for objects in collision 
        objNames = GetDarpaObjectNamesInScene( global_data, whichmanip, ['darpatable'] )
        for objName in objNames:
            if not orEnv.GetKinBody(objName):
                rospy.logerr( "  AttachObject: %s is not in the Openrave environment" % objName )
            else:
                collision = orEnv.CheckCollision(global_data.robot,global_data.orEnv.GetKinBody(objName))
                if collision:
                    cstr = "Yes"
                else:
                    cstr = "No"
                rospy.loginfo( "  AttachObject: checking for collision between robot and %s : %s" % ( objName, cstr ) )
                if collision:
                    contactObjName = objName
                    break

    #pdb.set_trace()
    success, objectPose = GetObjectPose( global_data, StripIndex( contactObjName ) )
    T0_obj = PoseToTransform( objectPose )
    rospy.loginfo( 'T0_obj (%s): %s' %( contactObjName, TranToStr( T0_obj ) ) )

    success = 0
    reason = "Grasp failed"
    if bool( contactObjName ):
        # check if object is held by inspecting finger pose 
        fdofs = robot.GetActiveDOFValues();
        rospy.loginfo( "Finger dofs = (%.3f %.3f %.3f)" % ( fdofs[0], fdofs[1], fdofs[2]) )
        maxfdofs = pi - 5 * pi / 180 
        if ( fdofs[0] > maxfdofs and fdofs[1] > maxfdofs and fdofs[2] > maxfdofs ) :
            global_data.probs_manip.SendCommand('releaseall')
            orEnv.Remove(orEnv.GetKinBody(contactObjName))
            success = 0
            reason = "Hand is closed without object"
            rospy.loginfo( "Error: %s" % reason )
            return success, reason
        
        success = 1
        reason = "GraspSucceeded"
        global_data.probs_manip.SendCommand('GrabBody name %s'%(contactObjName))
        #objName = StripIndex( contactObjName )
        #pdb.set_trace()
        success, reason = ComputeAndSetExtraMass( global_data, whichmanip, contactObjName )
        if not success:
            rospy.logerr( reason )
            # even if we are not able to set mass, it is OK not an unrecoverable error 
            success = 1

        robot.SetActiveDOFs(global_data.armdofs[whichmanip])

    return success, reason

"""
Close Fingers and then grasp object 
Uses position move which will keep applying pressure 
"""
def GraspObject( global_data, whichmanip, targetName ):
     # Grip the object tightly
     HandPositionMove([3.14, 3.14, 3.14, global_data.currentGrasp.final_grasp_dof[3]])
     time.sleep(2.0)
     success, reason = AttachObject( global_data, whichmanip, targetName )

     return success, reason



def MoveStraightForward( global_data, whichmanip, distance, blockFlag=True ):    
    """
    Perform pregrasp approach - nonblocking - must wait on movement elsewhere
    distance - how far to attempt to move
    blockFlag - whether to wait for completion of the move before returning.

    Returns success, reason
    """
    rospy.logdebug( "  Moving hand by %.3fm (forward)" % distance )
    manip = global_data.robot.GetManipulators()[whichmanip]
    T0_ee = mat(manip.GetEndEffectorTransform())
    pushdir = T0_ee[0:3][:,2].T
    success, reason, distMoved = MoveStraight( whichmanip, global_data, pushdir, distance, 0, blockFlag )
    return success, reason, distMoved



def MoveStraight(whichmanip, global_data, pushdir, targDist, breakOnCollision,
   blockFlag=True, simfile=None, cbirrt=False, guarded=False):
    """
    Move along pushdir from the current position. If the break on collision flag 
    is set, fail to plan a motion if it results in collision.  Blocks if
    blockFlag is true, otherwise, is nonblocking.

    Returns success, reason
    """

    if simfile != None:
        rospy.logdebug( "  Moving hand by %.3fm along (%s)" % ( targDist, pushdir) )

    pushdirstring = Serialize1DMatrix(mat(pushdir))
    rospy.logdebug( "  Moving hand by %.3fm along (%s)" % ( targDist, pushdirstring) )

    orEnv = global_data.orEnv
    robot = global_data.robot
    manip = robot.GetManipulators()[whichmanip]
    robot.SetActiveManipulator(whichmanip)
    robot.SetActiveDOFs(global_data.armdofs[whichmanip])

    # Get the manipulator's current position in simulation (for backup)
    backupconfig = robot.GetActiveDOFValues()
    
    # If we're appending to an existing simfile, put the arm in the last position
    if simfile != None:
        try:
            fin = open(simfile,'r')
            lines = fin.readlines()
            if len(lines) >= 1:
               last_line = lines[-1].split()
               robot.SetActiveDOFValues([float(last_line[ji]) for ji in global_data.armdofs[whichmanip]])
            fin.close()
        except IOError:
            pass
   
    # Get initial end-effector pose
    T0_ee = mat(manip.GetEndEffectorTransform())
    
    # Calculate plan into plan_filename, plan_lastconfig
    if cbirrt:
        plan_lastconfig = robot.GetActiveDOFValues() # By default, go nowhere
        
        H_world_ee = robot.GetActiveManipulator().GetEndEffectorTransform()
        # 'object frame w' is at ee, z pointed along direction to move
        H_world_w = cd_kin_H_from_op_diff(H_world_ee[0:3,3], pushdir)
        H_w_ee = dot(invert_H(H_world_w), H_world_ee)
        
        TSRchains = ''
        # Serialize TSR string (goal)
        Hw_end = eye(4)
        Hw_end[2,3] = targDist
        TSRstring = TSR.SerializeTSR(robot.GetActiveManipulatorIndex(), 'NULL', dot(H_world_w,Hw_end), H_w_ee, mat([0]*12))
        TSRchains += TSR.SerializeTSRChain(0,1,0,1,TSRstring,'NULL',array([]))
        # Serialize TSR string (whole-trajectory constraint)
        Bw = mat([0.0]*12)
        Bw[0,4] = min(0.0,targDist)
        Bw[0,5] = max(0.0,targDist)
        TSRstring = TSR.SerializeTSR(robot.GetActiveManipulatorIndex(), 'NULL', H_world_w, H_w_ee, Bw)
        TSRchains += TSR.SerializeTSRChain(0,0,1,1,TSRstring,'NULL',array([]))
        
        # Run the planner
        resp = global_data.probs_cbirrt.SendCommand('RunCBiRRT smoothingitrs 250 psample 0.1 timelimit 1.0 ' + TSRchains)
        
        # If success, get last config into plan_lastconfig
        if int(resp) != 1:
            'cbirrt planner failed!'
            pass
        else:
            try:
                fin = open('cmovetraj.txt','r')
                lines = fin.readlines()
                if len(lines) >= 1:
                   last_line = lines[-1].split()
                   plan_lastconfig = [float(last_line[1+ji]) for ji in global_data.armdofs[whichmanip]]
                fin.close()
            except IOError:
                pass
         
        # Get the trajectory
        plan_filename = 'cmovetraj.txt'
        
    else:
        # Always have the movement targDist be positive, instead just invert the push direction
        if targDist < 0:
            pushdir = -pushdir
            targDist = -targDist
        liftarmsteps = math.floor(targDist/.003)
        
        # Try to move along the approach direction
        resp = global_data.probs_manip.SendCommand('LiftArmGeneralIK breakoncollision %d exec 0 minsteps %d maxsteps %d direction %s'%(breakOnCollision, liftarmsteps, liftarmsteps, Serialize1DMatrix(mat(pushdir))))
        plan_filename = 'liftarmtraj.txt'
        plan_lastconfig = str2num(resp)
    
    # Put the robot in the end configuration, evaluate success
    robot.SetActiveDOFValues(plan_lastconfig)
    Final_ee = mat(manip.GetEndEffectorTransform())
    Tdiff = linalg.inv(T0_ee)*Final_ee
    distMoved = sqrt(Tdiff[0][:,3]*Tdiff[0][:,3] + Tdiff[1][:,3]*Tdiff[1][:,3] + Tdiff[2][:,3]*Tdiff[2][:,3])            
    distMoved = matrix( distMoved )[0,0]
    if distMoved < targDist - .001:
        rospy.logdebug( '  IK solution can only move %.3fm.'%(distMoved) )
        success = 0
        reason = "Failed to compute IK solution"
        #return 0, "Failed", distMoved
    else:
        rospy.logdebug( 'Target config is IK solution (distance=%fm).'%(distMoved) )
        success = 1
        reason = "Done"
        
    # Reset simulation
    robot.SetActiveDOFValues(backupconfig)
    
    # Save/Run trajectory as requested
    if simfile != None:
        # Append the trajectory to the simfile
        fin = open(plan_filename, 'r')
        fout = open(simfile, 'a')
        fin.readline()
        fin.readline()
        for line in fin:
            fout.write(line.split(' ',1)[1])
        fin.close()
        fout.close()
    else:
        # run trajectory
        orEnv.LockPhysics(False)
        rospy.logdebug('running trajectory')
        if guarded:
            TareForceTorque()
            SendToController(global_data, "stop_next_traj_on_force_input 1")
        RunBlendedTrajectory(global_data, 0, plan_filename, whichmanip)
        rospy.logdebug("trajectory executed")
        if blockFlag:
            # the caller is responsible for relocking the physics if they are not waiting
            success, reason = WaitForRobot(global_data, 1)
            if not success:
                rospy.logwarn("robot not done")
                orEnv.LockPhysics(True)
                return 0,"Robot not done:%s .  Moved %d"%(reason, distMoved), distMoved
            orEnv.LockPhysics(True)
    rospy.logdebug("returning")
    
    return success, reason, distMoved

def WriteSimToTrajFile(simfile, trajfile):
   fin = open(simfile, 'r')
   fout = open(trajfile, 'w')
   lines = fin.readlines()
   fout.write('\n')
   fout.write( '%d %d 76\n' % (len(lines), len(lines[0].split())-7) )
   time = 0.0
   for line in lines:
      fout.write('%f %s' % (time,line))
      time += 0.01
   fin.close()
   fout.close()



def MoveStraightGuarded(global_data, whichmanip, pushdir, distance,
    b_use_force_guard = False, b_use_stall_guard = False, monitor = None, threshold = 1.0, time_delay = None, sample_delay = None, b_change_arm_speed = True):
    """Most flexible guarded motion command
    If no monitor is given, the caller is allowed to pass in parameters to construct a local ActionMonitor.GuardedFTMotion monitor.
    See GuardedFTMotion for details

    This call never breaks on contact in the planner and ALWAYS BLOCKS
    
    @param b_use_force_guard - boolean flag to determine whether to activate the lowest level, most sensitive force torque monitor.
    This monitor watches for 8 samples of the force torque sensor being above some threshold in the -Z direction of the palm
    @param monitor - A parameter that allows th user to pass in a custom monitor for this movement.  See ActionMonitor.py for details

    The following parameters are used to construct a local GuardedFTMotion object:
    @param threshold - maximum allowed force in the direction being moved.  Does not effect owd guard
    @param time_delay - if given, delays the beginning of the guard.  Does not effect owd guard
    @param sample_delay - if given, delays any warnings until after the specified number of samples over the threshold have been seen.
    @param b_change_arm_speeed - if true, the guarded motion is performed at the superslow speed
    """
    TareForceTorque()
    
    status = []

    if b_change_arm_speed:
        SetArmSpeedSuperSlow()
    
    if monitor == None and threshold > 0:
        T0_ee = global_data.robot.GetManipulators()[whichmanip].GetEndEffectorTransform()
        monitor = ActionMonitor.GuardedFTMotion(-pushdir, T0_ee, threshold, time_delay = time_delay, sample_delay = sample_delay)
    if b_use_force_guard:
        SendToController( global_data, "stop_next_traj_on_force_input 1")
    if b_use_stall_guard:
        SendToController( global_data, "stop_next_traj_on_stall 1")


    success, result, dist2 = MoveStraight(whichmanip, global_data, pushdir, distance, breakOnCollision = False, blockFlag = True )

    if monitor:
        monitor.close()

    if b_use_force_guard:
        SendToController( global_data, "stop_next_traj_on_force_input 0")

    if b_use_stall_guard:
        SendToController( global_data, "stop_next_traj_on_stall 0")

    if b_change_arm_speed:
        SetArmSpeedFast();

    return success, result, dist2
    
    

    



"""
' Perform guarded forward motion in palm direction
' distance - How far to attempt to move
"""
def GuardedApproachAlongPalm(whichmanip, global_data, distance, monitor = None, threshold = 1.0, time_delay = None, sample_delay = None):
    SetArmSpeedSuperSlow();
    #Currently functions by calling guarded move service - still migrating to actionlib
    TareForceTorque()
    status = []
    T0_ee = global_data.robot.GetManipulators()[whichmanip].GetEndEffectorTransform()[0:3,:3]
    if monitor == None:
        monitor = ActionMonitor.GuardedFTMotion(- T0_ee[:3,2],  T0_ee[:3,:3], threshold, time_delay = time_delay, sample_delay = sample_delay)
    success, result, dist2 = MoveStraightForward( global_data, whichmanip, distance, 1 )
    if not ClearPausedTrajectories():
        status =  "No contact made in guarded move forward", list()
    else:
        status = "Moved forward to contact"
    monitor.close()
            
    SetArmSpeedFast();
    return success, "%s: %s"%(result,status)


def GuardedApproachAlongPalmServiceBlock(whichmanip, global_data, distance, pushdir=None):
   SetArmSpeedSlow()
   if pushdir == None:
      manip = global_data.robot.GetManipulators()[whichmanip]
      T0_ee = mat(manip.GetEndEffectorTransform())
      pushdir = T0_ee[0:3][:,2].T
   breakOnCollision = 0
   (success, reason, distmoved) = MoveStraight(whichmanip, global_data, pushdir, distance, breakOnCollision,
      blockFlag=True, simfile=None, cbirrt=False, guarded=True)
   FreeChainList = [1,1,1,1]
   SetArmSpeedFast()
   return success, FreeChainList


'''
'''

def GuardedApproachAlongPalmService(whichmanip, global_data, distance, pushdir = []):
    SetArmSpeedSlow();
    
    orEnv = global_data.orEnv
    robot = global_data.robot
    manip = robot.GetManipulators()[whichmanip]
    
    robot.SetActiveManipulator(whichmanip)
    robot.SetActiveDOFs(global_data.armdofs[whichmanip])
    
    T0_ee = mat(manip.GetEndEffectorTransform())
    if len(pushdir) == 0:
        pushdir = T0_ee[0:3][:,2].T
    
    #Get the manipulator's current position in simulation.
    backupconfig = robot.GetActiveDOFValues()

    #always have the movement targDist be positive, instead just invert the push direction
    if distance < 0:
        pushdir = -pushdir
        distance = -distance
    backupsteps=math.floor(distance/.003)
    breakOnCollision = 0
    #try to move along the approach direction
    resp = global_data.probs_manip.SendCommand('LiftArmGeneralIK breakoncollision %d exec 0 minsteps %d maxsteps %d direction %s'%(breakOnCollision, backupsteps, backupsteps, Serialize1DMatrix(mat(pushdir))))
    
    #If we were unable to go the appropriate targDist, fail.
    maybeconfig = str2num(resp)
    robot.SetActiveDOFValues(maybeconfig)
    newconfig = robot.GetActiveDOFValues()
    
    Final_ee = mat(manip.GetEndEffectorTransform())
    Tdiff = linalg.inv(T0_ee)*Final_ee
    distMoved = sqrt(Tdiff[0][:,3]*Tdiff[0][:,3] + Tdiff[1][:,3]*Tdiff[1][:,3] + Tdiff[2][:,3]*Tdiff[2][:,3])            
    distMoved = matrix( distMoved )[0,0]
    if distMoved < distance - .003:
        rospy.loginfo( '  IK solution can only move %.3fm.'%(distMoved) )
        success = 0
        reason = "Failed to compute IK solution"
        #return 0, "Failed", distMoved
    else:
        rospy.loginfo( 'Target config is IK solution (distance=%fm).'%(distMoved) )
        success = 1
        reason = "Done"
    # reset simulation
    robot.SetActiveDOFValues(backupconfig)
    rospy.loginfo( 'starting guarded move...' )
    pdb.set_trace()

     # run trajectory
    orEnv.LockPhysics(False)
    SetGuardedMoveSrv = rospy.ServiceProxy('/right/owd/guardedmove',GuardedMove)
    req = Joints()
    req.j = maybeconfig
    res = SetGuardedMoveSrv(req)
    success = res.ok
    FreeChainList = [1,1,1,1]

    #relock physics
    global_data.orEnv.LockPhysics(True)

    SetArmSpeedFast()
    
    return success, FreeChainList


'''
Return index of max value
'''
def GetMinIndex( val ):
    if val[0] < val[1]:
        if val[0] < val[2]:
            return 0
        else:
            return 2
    else:
        if val[1] < val[2]:
            return 1
        else:
            return 2


'''
Function to move persistently in a given direction
Uses default values for persistent motion
'''
def MovePersistent( global_data, whichmanip, pushdir, pushdist, sim=False, cbirrt=False ):
    lat_range = arange( 15, 45.1 )
    lon_range = arange( 0, 360, 30 )
    if cbirrt:
        lat_range = arange( 15, 45.1, 5 )
        lon_range = arange( 0, 360, 60 )
    return MovePersistentRange( global_data, whichmanip, pushdir, pushdist,
            lat_range, lon_range, sim=sim, cbirrt=cbirrt )

'''
Function to move persistently in a given direction
The directions along which IK solutions are computed are given by lat_range and
lon_range (which are in degrees)

The lon_range is the longitudinal directions: recommended values are 0:360 with
different step sizes. 

The lat_range is the latitudinal directions: recommended values depend on how much deviation
from the original direction is allowed. recommended from 10:45
'''
def MovePersistentRange( global_data, whichmanip, pushdir, pushdist, lat_range, lon_range, sim=False, cbirrt=False ):

    block = True
    simfile = None
    if sim: # Clear simulated accumulation file
        simfile = 'simfile.txt'
        f = open(simfile,'w')
        f.close()
    
    rospy.logdebug( "  Moving hand (%.3fcm)..." % pushdist )
    success, reason, dist = MoveStraight( whichmanip, global_data, pushdir,
            pushdist, 0, block, simfile=simfile, cbirrt=cbirrt )
    rospy.loginfo( "  moved %.3f (total %.3f of %.3f)" % ( dist, dist, pushdist ) )

    threshold_pushdist = pushdist - 0.0031
    # If pushdist was almost achived, declare success
    if dist >= threshold_pushdist:
        success = 1
    else:

        # x and y are orthonormal to pushdir
        ix = GetMinIndex( abs( pushdir ) )
        x = array( ( 0, 0, 0 ) )
        x[ix] = 1
        x = x - pushdir * dot( pushdir, x )
        y = cross( pushdir, x )
 
        for th_lat in lat_range * pi / 180:
            sth = sin( th_lat )
            cth = cos( th_lat )
            for th_lon in lon_range * pi / 180:
                pushdir2 = cth * pushdir + sth * (x * cos( th_lon ) + y * sin( th_lon ) )
                pushdir2 = pushdir2 / sqrt( dot( pushdir2, pushdir2 ) )
                costheta = dot( pushdir, pushdir2 )
                pushdist2 = ( pushdist - dist ) / costheta
                success, reason, dist2 = MoveStraight( whichmanip, global_data,
                        pushdir2, pushdist2, 0, block, simfile=simfile, cbirrt=cbirrt )
                dist = dist + dist2 * costheta
                if dist2 > 0.0031:
                    rospy.loginfo( "  moved %.3f (total %.3f of %.3f) along (lat=%3.0f, lon=%3.0f)" % ( 
                        dist2, dist, pushdist, th_lat * 180/pi, th_lon * 180/pi ) )
 
                if dist >= threshold_pushdist:
                    break
            if dist >= threshold_pushdist:
                break
    
    if sim: # If simulated, actually run the trajectory (since its not run in MoveStraight)
        WriteSimToTrajFile(simfile, 'simtraj.txt')
        # run trajectory
        global_data.orEnv.LockPhysics(False)
        rospy.logdebug('running trajectory')
        RunBlendedTrajectory(global_data, 0, 'simtraj.txt', whichmanip)
        rospy.logdebug("trajectory executed")
        if block:
            # the caller is responsible for relocking the physics if they are not waiting
            w_success, w_reason = WaitForRobot(global_data, 1)
            if not w_success:
                rospy.logwarn("robot not done")
                global_data.orEnv.LockPhysics(True)
                return 0,"Robot not done:%s .  Moved %d"%(w_reason, dist), dist
            global_data.orEnv.LockPhysics(True)
    
    return success, reason, dist



def MoveDirectionFromGrasp( grasp ):
    """
    Compute the move direction and distance from grasp.pre_grasp_pose to
    grasp.final_grasp_pose
    """
    T0_pre = PoseToTransform( grasp.pre_grasp_pose )
    T0_fin = PoseToTransform( grasp.final_grasp_pose )
    movedir = T0_fin[0:3,3] - T0_pre[0:3,3] 
    movedist = sqrt( dot( movedir, movedir ) )
    movedir = movedir / movedist
    return movedir, movedist



def MoveToFinal( global_data, whichmanip, grasp, movetype, extradist=0. ):
    """
    This is a generic function that will move from the end-effector from
    pre_grasp_pose  to final_grasp_pose. The hand is moved until
    1. final_grasp_pose + extradist is reached, or
    2. contact is made with an object (if movetype == to_contact)
    The above two cases will return success = 1 and the distance moved.

    If the motion fails due to IK failure or some other reason (stall), success
    = 0. 
    """
    rospy.loginfo( " "  )
    rospy.loginfo( "Moving to final " )
    rospy.loginfo( "  move_type = %s" % MoveType.toStr( movetype )  )
    #pdb.set_trace();

    rospy.loginfo( "  preshaping hand with dof: %s", grasp.per_grasp_dof )
    HandPositionMove( grasp.pre_grasp_dof )

    # move forward 
    time.sleep( 1 )
    movedir, movedist = MoveDirectionFromGrasp( grasp )
    rospy.loginfo( "  moving to final position (movedir= %s, movedist= %.3f, extradist= %.3f )" %
            ( movedir, movedist, extradist ) )

    if movetype == MoveType.to_final:
        success, reason, distmoved = MoveStraight( whichmanip, global_data, movedir, movedist, 0 )
    else:
        b_use_force_guard = True
        success, reason, distmoved = MoveStraightGuarded( global_data,
            whichmanip, movedir, movedist, b_use_force_guard, threshold = 0.5,
            sample_delay = 4 )

    rospy.loginfo( "  success=%d, distmoved=%.3f" % ( success, distmoved ) )

    return success, reason, distmoved 


def MoveFromFinal( global_data, whichmanip, grasp, movetype, movedist2=0. ):
    """
    This is a generic function that will move from the end-effector from
    current position by movedist2. The direction is the direction of the grasp.
    The hand is moved until
    1. movedist is reached, or
    2. contact is made with an object (if movetype == to_contact)
    The above two cases will return success = 1 and the distance moved.

    If the motion fails due to IK failure or some other reason (stall), success
    = 0. 
    """

    # move forward 
    time.sleep( 1 )
    movedir, movedist = MoveDirectionFromGrasp( grasp )
    movedist = movedist2
    rospy.loginfo( "  moving to position (movedir= %s, movedist= %.3f )" %
            ( movedir, movedist ) )

    if movetype == MoveType.to_final:
        success, reason, distmoved = MoveStraight( whichmanip, global_data, movedir, movedist, 0 )
    else:
        b_use_force_guard = True
        success, reason, distmoved = MoveStraightGuarded( global_data,
            whichmanip, movedir, movedist, b_use_force_guard )

    rospy.loginfo( "  success=%d, distmoved=%.3f" % ( success, distmoved ) )

    return success, reason, distmoved 



def ExecuteFinal( global_data, whichmanip, grasp, applyforce ):
    """
    This function will perform the final grasping motion using 
    1. ApplyForce + close hand (in the direction of the grasp)
       - If ApplyForce fails, setstiffness is used
    2. Vanilla close hand
    """

    # Move back a bit (ensure there is no contact with table)
    movedir, movedist = MoveDirectionFromGrasp( grasp )
    success, reason, distmoved = MoveStraight( whichmanip, global_data,
            movedir, -0.01, 0 )

    # Tare force torque sensor (for both applyforce and graspverify)
    for r in range( 0, 3 ):
        rospy.loginfo( "Taring F/T" )
        TareForceTorque()
        time.sleep( 0.5 )

    success, reason = GraspVerifyInitialize( global_data, whichmanip )

    final_grasp_dof = copy.deepcopy( grasp.final_grasp_dof )
    SetHandSpeedScalar( 0.25 )

    if applyforce.flag:
        # Try ApplyForce a few times 
        success = 0
        forcedir = WorldToWam0( movedir )
        forceval = applyforce.value
        rospy.loginfo( "Applying force (forcedir= %s, forceval=%.3f)" % ( forcedir, forceval ) )
        attempts = 0
        while success == 0 and attempts < 5:
            attempts += 1
            success, reason = ApplyForceSrv( forcedir, forceval )
            if success > 0:
                rospy.loginfo( '  ApplyForce: succeeded' );
                break
            else:
                rospy.logwarn( '  ApplyForce: failed (%s)' % reason );
        time.sleep( 3 )
    
        if success:
#            # If ApplyForce succeeds, close fingers in steps 
#            # Wait for hand to settle down on table
#            time.sleep( 2 )
#
#            # Close to slightly over maximal distention and wait
#            for angle in [1.2, 1.5]:
#                final_grasp_dof[0:3] = angle * [1,1,1]
#                HandPositionMove( final_grasp_dof )
#                time.sleep( 1 )
#
#            HandPositionMove( grasp.final_grasp_dof )

            # Stop applying downward force 
            rospy.loginfo( "Stopping force" )
            StopForceSrv()
        else:
            # If ApplyForce fails, try in gravity comp mode 
            rospy.logwarn( "ApplyForce failed: Will try in gravity compensation mode" )
            SetStiffness( 0.01)
            HandPositionMove( grasp.final_grasp_dof )

    else:
        HandPositionMove( grasp.final_grasp_dof )

    SetHandSpeedScalar( 0.25 )

    return success, reason


    
'''
Move forward until contact with table, lift up a bit, and close fingers 
- Assume EE is facing down toward table and object, with fingers open 
- Half close fingers 
'''

def ContactWithTableClose( global_data, whichmanip, targetName ):
    rospy.loginfo( " "  )
    rospy.loginfo( "--"  )
    rospy.loginfo( "Moving to contact with table"  )

    # Move toward table to locate EE wrt to table surface
    manips = global_data.robot.GetManipulators()
    T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_e, ya_e, za_e, t_e = MatrixToAxes( T0_ee )

    #get ik solutions and see if you can back up from them
    pushdir = T0_ee[0:3][:,2].T # za_e ?
    
    # get table pose
    gotTablePose, tablePose = GetObjectPose( global_data, "darpatable" )
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
    pN = za_t
    pD = -dot( za_t, t_t )
    xa_p, ya_p, za_p, t_p = MatrixToAxes( TSRUtil.E2P( T0_ee ) )
    d_from_table = dot( za_t, t_p ) + pD
    rospy.loginfo( "  End effector is %fm from table" % d_from_table )

    # Plan to move to maximal distention from table + tolerance
    d_desired = 0.13 + 0.05
    d_move = d_from_table - d_desired
    rospy.loginfo( "  Planned move %.3fm" % ( d_move ) )

    breakOnCollision = 1
    block = True
    MoveStraight( whichmanip, global_data, pushdir, d_move, breakOnCollision, block )

    HandPositionOpenMaximalDistention()
    time.sleep(2.0)
    #pdb.set_trace()
    
    dstep = 0.10
    breakOnCollision = 1
    block = True

    MoveStraight( whichmanip, global_data, pushdir, dstep, breakOnCollision, block )
    T0_ee_i = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_i, ya_i, za_i, t_i = MatrixToAxes( T0_ee_i )
    dt = t_i - t_e
    dd = sqrt( dot( dt, dt ) )
    rospy.loginfo( "  Moved to (%.3f %.3f %.3f), d = %.3f" % ( t_i[0], t_i[1], t_i[2], dd ) )
    #pdb.set_trace()
        
    rospy.loginfo( "  Backing up just a bit" )
    dstep = 0.008
    MoveStraight( whichmanip, global_data, -pushdir, dstep, 0, block )
    
    # Initialize grasp verify service
    success, reason = GraspVerifyInitialize( global_data, whichmanip )

    # Full close fingers 
    #HandClose( global_data, whichmanip )
    HandPositionClose()
    time.sleep( 2.0 )
    rospy.loginfo( "  Pushing object into table" )
    # Try push object against table and into grasp 
    dstep = 0.0075
    MoveStraight( whichmanip, global_data, pushdir, dstep, 0, block )
    time.sleep( 1.0 )
    SetArmSpeedFast()
    AttachObject( global_data, whichmanip, targetName )
 
    rospy.loginfo( "Lifting up object..." )
    dstep = 0.08
    MoveStraight( whichmanip, global_data, -pushdir, dstep, 0, block )

    # Confirm grasp using grasp verify service
    success, reason = GraspVerifyCheckWithHandPose( global_data, whichmanip )
    rospy.loginfo( "Grasp verification returned %d (%s)" % ( success, reason ) )

    return success, reason


'''
Move forward until contact with table, and close fingers while applying downward
force. 
- Assume EE is facing down toward table and object, with fingers open 
- Half close fingers 
'''


def ContactWithTableApplyForceAndClose( global_data, whichmanip, targetName ):

    #get the grasp parameters
    gp = global_data.grasp_parameter
   
    rospy.loginfo( " "  )
    rospy.loginfo( "--"  )
    rospy.loginfo( "Moving to contact with table (apply force)"  )

    # Open hand wide first
    SetHandSpeedScalar( 1.5 )

    # Open to semi-caging position before moving down
    HandPositionMove( global_data.currentGrasp.pre_grasp_dof )
    time.sleep(1.0)
    for t in arange( 3 ):
        robsuccess, robstatus = UpdateRobot( global_data, whichmanip )
        if StatusStopEverything(robstatus):
            return 0, robstatus
        time.sleep( 0.5 )

    # get EE pose
    manips = global_data.robot.GetManipulators()
    T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_e, ya_e, za_e, t_e = MatrixToAxes( T0_ee )

    # get table pose
    gotTablePose, tablePose = GetObjectPose( global_data, "darpatable" )
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )

    # compute distance to be moved to be in desired starting position 
    pushdir = za_e 
    
    pN = za_t
    pD = -dot( za_t, t_t )
    xa_p, ya_p, za_p, t_p = MatrixToAxes( TSRUtil.E2P( T0_ee ) )
    d_from_table = dot( za_t, t_p ) + pD
    rospy.loginfo( "Palm is %fm from table" % d_from_table )

    # Note: hand is around 13 cm from the table in maximum distention 
    #       hand is around 5 cm from the table in minimal distention 

    # move until contact with table 
    # ie, get close to table before calling apply force 
    # Move hand d_desired above table 
    d_maxdistention = 0.13
    d_offset = 0.10
    d_desired = d_maxdistention + d_offset
    d_move = d_from_table - d_desired
    rospy.loginfo( "  Planned move %.3fm" % ( d_move ) )

    rospy.loginfo( 'Trying to contact w/table' )
    d_move = d_offset + 0.10

    TareForceTorque()
    GuardedApproachAlongPalm(whichmanip, global_data, d_move, monitor = None, threshold = 3.0, time_delay = None, sample_delay = 3)
    time.sleep( 1 )
    d_back = -0.01
    rospy.loginfo( 'Moving back a bit (%.3f)' % d_back )
    GuardedApproachAlongPalm(whichmanip, global_data, d_back, monitor = None, threshold = 3.0, time_delay = None, sample_delay = 3)
    time.sleep( 1 )

    for r in range( 0, 3 ):
        rospy.loginfo( "Taring F/T" )
        TareForceTorque()
        time.sleep( 0.5 )

    # Initialize grasp verify service
    success, err = GraspVerifyInitialize( global_data, whichmanip )

    #pdb.set_trace()
    # Apply downward force (Tested well with 0.5 to 1.0N)
    downwardForce = 0.70
    forcedir = array( (-1,0,0) )

    success = 0
    attempts = 0
    while success == 0 and attempts < 5:
        attempts += 1
        rospy.loginfo( "Applying downward force" )
        success, reason = ApplyForceSrv( forcedir, downwardForce )
        if success > 0:
            rospy.loginfo( '  ApplyForce: succeeded' );
            break
        else:
            rospy.loginfo( '  ApplyForce: failed (%s)' % reason );
        time.sleep( 1 )

    # Wait for hand to settle down on table
    time.sleep( 2 )

    # Now slow down hand speed
    SetHandSpeedScalar( 0.15 )

    for angle in arange( 1, 1.5, 0.1 ):
        HandPositionMove( [angle, angle, angle, global_data.currentGrasp.final_grasp_dof[3]] )
        time.sleep( 0.25 )
        robsuccess, robstatus = UpdateRobot( global_data, whichmanip )
        if StatusStopEverything(robstatus):
            return 0, robstatus
        time.sleep( 0.5 )

    HandPositionMove( global_data.currentGrasp.final_grasp_dof, True)

    # Monitor finger pose XXX FIXME this does not work 
    for t in arange( 3 ):
        robsuccess, robstatus = UpdateRobot( global_data, whichmanip )
        if StatusStopEverything(robstatus):
            return 0, robstatus( global_data, whichmanip )
        time.sleep( 1.0 )

    # Fully close now and attach object
    SetHandSpeedScalar( 0.25 )
    GraspObject( global_data, whichmanip, targetName )

    # Stop applying downward force 
    rospy.loginfo( "Stopping downward force" )
    StopForceSrv()
    
    SetHandSpeedScalar( 0.8 )
    time.sleep(1.0)

    speed_profile = getattr(gp, 'transport_speed')
    SetArmSpeedRate(speed_profile[0], speed_profile[1])

    rospy.loginfo( 'Lifting up object' )
    success, reason, dist = liftUp(global_data, whichmanip, targetName)

    if not success:
        rospy.logerr( "Can only move %.3f" % (dist) )

    # Confirm grasp using grasp verify service
    FT_THRESHOLD = 1.0
    success, reason = GraspVerifyCheckFT( global_data, whichmanip, FT_THRESHOLD )
    if 'darpafloodlight' in targetName:
        rospy.loginfo( 'skip grasp verification since it is floodlight ')
        success = True
        reason = ''
    else:
        rospy.loginfo( "Grasp verification returned %d (%s)" % ( success, reason ) )

    return success, reason
    

'''
Baseline power grasp function 
Move forward to finalPose and close 
'''

def PowerGraspClose( global_data, whichmanip, targetName, finalPose, tablePose, liftObjectFlag=True ):

    #get the grasp parameters
    gp = global_data.grasp_parameter
    
    rospy.loginfo( "--" )
    rospy.loginfo( "Power-grasping object" )
    manips = global_data.robot.GetManipulators()
    T0_ee_init = mat(manips[whichmanip].GetEndEffectorTransform())
    T0_ee_final = mat( TSRUtil.G2E( PoseToTransform( finalPose ) ) )

    # if it is a floodlight, in powergrsap is used when the floodlight is stand vertically
    # now we change the spread angle and start apply force downward to better locate the hand height
    #pdb.set_trace()
    if "darpafloodlight" in targetName:
        # set the fingers velocities to be slow
        SetHandSpeedScalar( 1.5 )
        # move to a hand pose that helps we locate the palm relative height
        HandPositionMove( [0,0,0,2.9] )
        time.sleep(2.0)

        #start applying force downward
        force = 1.0
        forcedir = array( (0,0,-1) )
        startApplyForce(forcedir, force)

        # slow down the velocity
        SetHandSpeedScalar( 0.3 )
        # in case the first open of the spread angle was not very successful when the hand was closer to the table
        # instead of further from it
        HandPositionMove( [0,0,0,2.7] )
        time.sleep(3.0)
        
        # Stop applying downward force 
        rospy.loginfo( "Stopping downward force" )
        StopForceSrv()

        success, status = UpdateRobot( global_data, whichmanip )

            
    # computing the push
    t_f = array( T0_ee_final[0:3][:,3] ).flatten()
    t_i = array( T0_ee_init[0:3][:,3] ).flatten()
    pushval = t_f - t_i
    safety_dist = 0.02
    pushdist = sqrt( dot( pushval, pushval ) ) - safety_dist
    pushdir = pushval / pushdist

    if "darpafloodlight" in targetName:
        T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
        xa_c, ya_c, za_c, t_c = MatrixToAxes( T0_ee_curr )
        axis_45_degree = xa_c + za_c
        axis_norm = sqrt( dot(axis_45_degree, axis_45_degree) )
        pushdir = axis_45_degree / axis_norm
        pushdist = 0.08
        rospy.loginfo(" Pushdir: [%s] from [%s] with distance [%s]" % (pushdir, axis_45_degree, pushdist) )

    # hand preshape
    SetHandSpeedScalar( 1.2 )
    HandPositionMove( global_data.currentGrasp.pre_grasp_dof )
    time.sleep(2.0)
    #pdb.set_trace()
    
    # start from the pre-grasp position, move forward to final grasp position
    breakOnCollision = 0
    block = True
    rospy.loginfo( "Before move straight with distance: %s" % (pushdist) )
    MoveStraight( whichmanip, global_data, pushdir, pushdist, breakOnCollision, block )
    rospy.loginfo( "After move straight" )
    T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_c, ya_c, za_c, t_c = MatrixToAxes( T0_ee_curr )
    rospy.logdebug( "  T_ee initial (%.3f %.3f %.3f)" % ( t_i[0], t_i[1], t_i[2] ) )
    rospy.logdebug( "  T_ee final   (%.3f %.3f %.3f)" % ( t_f[0], t_f[1], t_f[2] ) )
    rospy.logdebug( "  T_ee current (%.3f %.3f %.3f)" % ( t_c[0], t_c[1], t_c[2] ) )
    #pdb.set_trace()
    rospy.logdebug( "  pushdir (%.3f %.3f %.3f), pushval = %.3f" % ( pushdir[0], pushdir[1], pushdir[2], pushdist ) )
 
    # Initialize grasp verify service
    success, err = GraspVerifyInitialize( global_data, whichmanip )

    # Full close fingers 
    HandPositionMove( global_data.currentGrasp.final_grasp_dof, True )
    time.sleep(3.0)
    
    AttachObject( global_data, whichmanip, targetName )
    robsuccess, robstatus = UpdateRobot( global_data, whichmanip )

    
    speed_profile = getattr(gp, 'transport_speed')
    SetArmSpeedRate(speed_profile[0], speed_profile[1])
    if liftObjectFlag:
        rospy.loginfo( "Lifting up object" )
        liftUp(global_data, whichmanip, targetName)
        #xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
        #pushdir = za_t
        #dstep = 0.15
        #MoveStraight( whichmanip, global_data, pushdir, dstep, 0, block )
 
    # Confirm grasp using grasp verify service
    success, reason = GraspVerifyCheckWithHandPose( global_data, whichmanip )
    if 'darpafloodlight' in targetName:
        rospy.loginfo(' skip grasp verification ')
        success = True
        reason = ''
    else:
        rospy.loginfo( "Grasp verification returned %d (%s)" % ( success, reason ) )

    return success, reason

'''
A simple version of move until contact and close the hand
supposed to work with power grasps and heavy objects
move to contact -> move back a little bit -> close the fingers
'''
def MoveUntilTouchAndClose(global_data, whichmanip, targetName, tablePose, liftObjectFlag = True):

    #get the grasp parameters
    gp = global_data.grasp_parameter
    
    manips = global_data.robot.GetManipulators()
    #pdb.set_trace()

    #pre-shape the hand
    SetHandSpeedScalar( 1.2 )
    HandPositionMove(global_data.currentGrasp.pre_grasp_dof)
    time.sleep(2.0)

    #retrieve move back distance
    move_back_dist = getattr(gp, 'move_back_distance')

    #object-specific action not covered in yamldb
    if "darpapelican" in targetName:
        success, pelicanPose = GetObjectPose( global_data, targetName)
        rospy.loginfo(" pelican case pose is %s " % (pelicanPose) )
        xa_c, ya_c, za_c, t_c = MatrixToAxes(  PoseToTransform(pelicanPose) )
        #when the pelican is laying down on the table, we want the palm to touch the table
        #so that the fingers can get into the handle from under the handle
        if ya_c[2] > 0.7:
            rospy.loginfo(" pelican is horizontally laying down on the table ")
            down_dir = array( (0,0,-1) )
            down_dist = 0.05
            success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, down_dist, down_dir )
            time.sleep(2.0)
            success, status = UpdateRobot( global_data, whichmanip )
            
            #move forward for a distance of 7 cm
            T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
            xa_c, ya_c, za_c, t_c = MatrixToAxes( T0_ee_curr )
            pushdir = za_c
            dist = 0.07
            block = True
            MoveStraight( whichmanip, global_data, pushdir, dist, 0, block )
            time.sleep(2.0)
            ClearPausedTrajectories()
            success, status = UpdateRobot(global_data, whichmanip)
            #move_back_dist = 0.02
        else:
            rospy.loginfo(" pelican is either sideways or vertically on the table ")
            HandPositionMove(0)
            time.sleep(1.0)
            #move_back_dist = 0.05
    
    #move to hit
    rospy.loginfo("guarded move forward start")
    distance_extra = 0.15
    success, FreeChainList = GuardedApproachAlongPalmServiceBlock(whichmanip, global_data, distance_extra )
    #success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, distance_extra )
    #time.sleep(3.0)
    if success != 1:
       errstr = "Failed to execute guarded move!"
       rospy.loginfo(errstr)
       return False, errstr
    success, status = UpdateRobot(global_data, whichmanip)
    rospy.loginfo("guarded move forward done")

    #move back a little bit
    T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_c, ya_c, za_c, t_c = MatrixToAxes( T0_ee_curr )
    pushdir = -za_c
    #if "darpaphonehandset" in targetName:
    #    move_back_dist = 0.03
    block = True
    rospy.loginfo("pelican back with %f along %s" % (move_back_dist, pushdir) )
    (success, reason, distmoved) = MoveStraight( whichmanip, global_data, pushdir, move_back_dist, 0, blockFlag=block )
    if success != 1:
       errstr = "Failed to execute back up, MoveStraight error:" + reason
       rospy.loginfo(errstr)
       return False, errstr

    #close the fingers
    finger_speed = getattr(gp, 'finger_close_speed')
    SetHandSpeedScalar(finger_speed)
    
    if 'darpapelican' in targetName:
        rospy.loginfo(" pelican close not fully ")
        new_dof = [global_data.currentGrasp.pre_grasp_dof[0] * 1.5,
                   global_data.currentGrasp.pre_grasp_dof[1] * 1.5,
                   global_data.currentGrasp.pre_grasp_dof[2] * 1.5,
                   global_data.currentGrasp.pre_grasp_dof[3] * 1.0]
        global_data.currentGrasp.final_grasp_dof = new_dof
        HandPositionMove(global_data.currentGrasp.final_grasp_dof)
    else:
        HandPositionMove(global_data.currentGrasp.final_grasp_dof, True)
        
    time.sleep(2.0)
    AttachObject( global_data, whichmanip, targetName )

    global_data.orEnv.LockPhysics(False)
    time.sleep(3.0)
    global_data.orEnv.LockPhysics(True)
    
    rospy.loginfo(' about to update robot' )
    success, status = UpdateRobot(global_data, whichmanip)
    if success:
        rospy.loginfo(' update robot succeeded' )
    else:
        rospy.loginfo(' update robot not succeeded' )

    arm_speed = getattr(gp, 'transport_speed')
    SetArmSpeedRate(arm_speed[0], arm_speed[1])
    if liftObjectFlag:
        liftUp(global_data, whichmanip, targetName)
 
    # Confirm grasp using grasp verify service
    #success, reason = GraspVerifyCheckWithHandPose( global_data, whichmanip )
    #rospy.loginfo( "Grasp verification returned %d (%s)" % ( success, err ) )

    return True, ''

'''
A function call dedicated to execute a grasp for fetching a drill without turning it on
'''
def FetchWithControlledFingerDOF( global_data, whichmanip, grasp, targetName, tablePose, liftObjectFlag = True):
    success = True
    reason = ''

    SetHandSpeedScalar( 1.5 )

    #prepare for the grasp verify later
    for r in range( 0, 3 ):
        rospy.loginfo( "Taring F/T" )
        TareForceTorque()
        time.sleep( 0.5 )

    # Initialize grasp verify service
    success, err = GraspVerifyInitialize( global_data, whichmanip )

    preshapeangle = 0.9
    HandPositionMove( [preshapeangle, preshapeangle, preshapeangle, grasp.final_grasp_dof[3]] )
    time.sleep(1.5)
    #move to the drill
    distance_extra = interPoseDistance(mat(pm.toMatrix(pm.fromMsg(grasp.final_grasp_pose))), mat(pm.toMatrix(pm.fromMsg(grasp.pre_grasp_pose))))

    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'
    
    #record the initial position
    manips = global_data.robot.GetManipulators()
    T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_e, ya_e, za_e, start_position = MatrixToAxes( T0_ee )

    #move forward to hit the drill
    rospy.loginfo('  want to move forward for %f: , start location is: %s' % (distance_extra, start_position))
    success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, distance_extra)
    #wait until guarded move action finishes
    time.sleep(3.0)
    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'

    #move back a little bit so that tare force will succeed
    dist = -0.005
    success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, dist)
    time.sleep(2.0)
    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'
    
    #now tare force and close the fingers very gently and try to center the palm w.r.t the drill horizontally
    palm_force_dir = array([1,0,0])
    force_magnitude = 0
    rospy.loginfo(" Taring the F/T ")
    TareForceTorque()
    for i in range(3):
        success, reason = ApplyForceInPalmDir( global_data, whichmanip, palm_force_dir, force_magnitude )
        if success:
            break
        TareForceTorque()
        time.sleep(1)
        
    if not success:
        rospy.loginfo( " ApplyForceInPalmDir is not successful (%s)" % reason)
    SetHandSpeedScalar( 0.5 )
    HandPositionMove([1.6,1.6,1.6,grasp.final_grasp_dof[3]])
    time.sleep(2.0)
    StopForceSrv()
    time.sleep(1.0)
    SetHandSpeedScalar( 1.5 )
    for i in range( 2 ):
        HandPositionMove( [preshapeangle, preshapeangle, preshapeangle, grasp.final_grasp_dof[3]] )
        time.sleep(1.0)
    time.sleep(1.0)
    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'   

    #now move forward again in case we were too far away from the center of the drill
    dist = 0.05
    success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, dist)
    time.sleep(1.0)
    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'
    
    #now move down along -z direction so that the palm hits the knot at the bottom of the drill's handle
    if 0:
        pushdir = [0,0,-1]
        dist = 0.08
        success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, dist, pushdir)
        #wait until guarded move action finishes
        time.sleep(2.0)
        
        success, status = UpdateRobot(global_data, whichmanip)
        if not success:
            return False, 'robot not updated or actions cancelled'
        ClearPausedTrajectories()

        dist = 0.03
        pushdir = [0,0,1]
        success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, dist, pushdir)
        time.sleep(1.0)
        
        ## success, status = UpdateRobot( global_data, whichmanip )
        success, status = UpdateRobot(global_data, whichmanip)
        if not success:
            return False, 'robot not updated or actions cancelled'
    
    # get the current position of the palm
    T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
    objectInWorld = mat(global_data.orEnv.GetKinBody(targetName).GetTransform())

    # ideally, we only adjust the object according to the palm's x,y axes
    # but considering the object may not be perfectly vertical, we may want to consider z-axis
    # now this is good enough
    objectNewLocation = dot(T0_ee,array([0,0,0.17,1]).reshape(4,1))
    objectInWorld[0,3] = objectNewLocation[0,0]
    objectInWorld[1,3] = objectNewLocation[1,0]
    
    #rospy.loginfo( " after: %s" % objectInWorld )
    ## pdb.set_trace()
    
    global_data.orEnv.GetKinBody(targetName).SetTransform(array(objectInWorld))

    #close the fingers that are not touching the trigger
    HandPositionMove(grasp.final_grasp_dof)
    success, reason, positions = WaitForHand(whichmanip)

    if grasp.final_grasp_dof[0] < 2.5:
        global_data.fingerOnTrigger = 0
    else:
        global_data.fingerOnTrigger = 1
        
    ## time.sleep(0.2)
    ## success, status = UpdateRobot( global_data, whichmanip )    
    success, status = UpdateRobot(global_data, whichmanip)
    #pdb.set_trace()

    AttachObject( global_data, whichmanip, targetName )

    if not success:
        return False, 'robot not updated or actions cancelled'

    if liftObjectFlag:
        rospy.loginfo( "  Lifting up object" )
        xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
        pushdir = za_t
        dstep = 0.15
        block = True
        MoveStraight( whichmanip, global_data, pushdir, dstep, 0, block )

    # Confirm grasp using grasp verify service
    FT_THRESHOLD = 1.0
    success, reason = GraspVerifyCheckFT( global_data, whichmanip, FT_THRESHOLD )        
    
    return success, reason
'''
A function to move the drill forward to get into contact with the drilling surface
and then apply force along the normal of the surface
'''
def ApplyDrilling(whichmanip, global_data, targets):

    rospy.loginfo( " "  )
    rospy.loginfo( "--"  )
    rospy.loginfo( "Begin drilling action"  )

    #1: move to touch the surface
    success, blockPose = GetObjectPose( global_data,  targets.split()[0])
    T_block_in_world = PoseToTransform( blockPose )
    blocknormal = T_block_in_world[0:3][:,2].T
    pushdir = [-blocknormal[0], -blocknormal[1], -blocknormal[2]]
    distance = 0.2
    rospy.loginfo("  Move forward until touching the surface with pushdir: %s" % pushdir)
    #let's wai for 2 second so that the drill stops shaking!
    time.sleep(2.0)
    #set it to be slow so that the arm does not jerk, but it still does it
    #jerking could confuse the force/torque sensor and makes it believe it
    #has hit something while it does not
    SetArmSpeedRate(0.05, 0.05)
    for i in range(3):
        #success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, distance, pushdir)
        TareForceTorque()
        GuardedApproachAlongPalm(whichmanip, global_data, distance, monitor = None, threshold = 3.0, time_delay = None, sample_delay = 3)
        if not success:
            rospy.loginfo('   Not move until contact')
        print 'begin to touch the drilling surface now'
        #wait until the guarded move stops, not really necessary, but for safe!
        time.sleep(3.0)
    print 'should be touching the drilling surface now'

    #2.b now only use MoveStraight to do the drilling
    breakOnContact = False
    block = True
    success, status = UpdateRobot(global_data, whichmanip)
    if not success:
        return False, 'robot not updated or actions cancelled'
    print 'begin to move down'
        
    #2.1 update the robot position and wait until we made a 2.5 cm progress
    manips = global_data.robot.GetManipulators()
    T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_e, ya_e, za_e, start_position = MatrixToAxes( T0_ee )
    drill_done, reason, dist = MoveStraight(whichmanip, global_data, pushdir, 0.08, breakOnContact, block)
    current_position = []
    dist = 0
    drill_done = False
    steps_to_try = 30
    for t in range(0, steps_to_try):
        time.sleep( 1.0 )
        success, status = UpdateRobot(global_data, whichmanip)
        if not success:
            return False, 'robot not updated or actions cancelled'
        
        T0_ee = mat(manips[whichmanip].GetEndEffectorTransform())
        xa_e, ya_e, za_e, current_position = MatrixToAxes( T0_ee )
        dist = linalg.norm(current_position - start_position)
        rospy.loginfo("distance proceeded: %f" % (dist) )
        if dist > 0.05:
            drill_done = True
            break
        elif t == steps_to_try - 1: #index starts from zero to steps_to_try - 1
            break
        
    success, status, dist_moved = MoveStraight(whichmanip, global_data, blocknormal, linalg.norm(current_position - start_position) + 0.1, breakOnContact, block)

    if StatusStopEverything(status):
        return success, status


    print 'reason: ' + reason
    
    if drill_done:
        rospy.loginfo("Drilling action is done successfully, now let's pull the drill out")
        reason = "Succeeded drilling " + '%f' % dist
    else:
        rospy.loginfo("Drilling action failed, now let's pull the drill out")
        reason = "Did not pushed deep enough"
    SetArmSpeedFast()

    return drill_done, reason


'''
Approach to contact powergrasp
Move forward to finalPose and close
'''
def MoveToContactPowerGrasp( global_data, whichmanip, targetName, currentGrasp, tablePose):
    FreeChainList = [1,1,1,1]
    rospy.loginfo( "Preshaping hand" )
    #pdb.set_trace()
    success, FreeChainList = GuardedHandMotion(whichmanip, global_data, currentGrasp.pre_grasp_dof, FreeChainList)              
    if not success:
        return 0,"Couldn't move forward"

    # Initialize grasp verify service
    success, reason = GraspVerifyInitialize( global_data, whichmanip )
    
    #find appropriate distance to move forward
    dist = interPoseDistance(mat(pm.toMatrix(pm.fromMsg(currentGrasp.final_grasp_pose))),
                             mat(pm.toMatrix(pm.fromMsg(currentGrasp.pre_grasp_pose))))
    # if arm is not yet in contact (and it should not be if pose error is small), move further forward
    rospy.loginfo( "Approaching object to contact" )
    distance_extra = -0.01
    success, FreeChainList = GuardedApproachAlongPalm(whichmanip, global_data, dist + distance_extra )


    # Move fingers closed until they make contact with the object
    rospy.loginfo( "Moving fingers to contact" )
    ffc = [2.2,2.2,2.2, currentGrasp.final_grasp_dof[0]]
    success, FreeChainList = GuardedHandMotion(whichmanip, global_data,  ffc, FreeChainList)

    # Tighten grip on object and attach object in simulation
    rospy.loginfo( "Closing Grasp" )
    # XXX FIXME shouldn't have to do this (check for finger closure
    # automatically)
    SetHandSpeedScalar( 1.5 )
    for i in range( 3 ):
        HandPositionClose()
        time.sleep( 1 )

    GraspObject( global_data, whichmanip, targetName )
    SetArmSpeedFast()

    lift_up_dist = 0.20
    rospy.loginfo( "Lifting up object by %.3f" % lift_up_dist )
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
    pushdir = za_t
    block = True

    success, reason, dist = MovePersistent( global_data, whichmanip, pushdir, lift_up_dist )
    rospy.loginfo( "Moved %.3f (total %.3f of %.3f)" % ( dist, dist, lift_up_dist ) )

    # Confirm grasp using grasp verify service
    success, reason = GraspVerifyCheckWithHandPose( global_data, whichmanip )
    rospy.loginfo( "Grasp verification returned %d (%s)" % ( success, reason ) )
    return success, reason



'''
Attach object to the robot 
Also SetExtra mass for link 7 for specific objects
Returns success, followed by reason  
'''
def ReGrabObject( global_data, whichmanip ):
    grabbedBodies = global_data.robot.GetGrabbed()
    #pdb.set_trace()
    for body in grabbedBodies:
        objName = body.GetName()

        rospy.loginfo( 'Releasing %s' % objName ) 
        global_data.robot.Release( body )
        rospy.loginfo( 'Grabbing %s' % objName ) 
        global_data.probs_manip.SendCommand( 'GrabBody name %s'%( objName ) )
        time.sleep( 0.1 )

    return 1, ""

'''
Detach object to the robot 
Returns success, followed by reason  
'''
def UnGrabObject( global_data, whichmanip ):
    grabbedBodies = global_data.robot.GetGrabbed()
    #pdb.set_trace()
    for body in grabbedBodies:
        objName = body.GetName()
        rospy.loginfo( 'Releasing %s' % objName ) 
        global_data.robot.Release( body )
        time.sleep( 0.1 )

    return 1, ""

'''
Put object down on table 
'''
# XXX FIXME first draft

def PutDownOnTable( global_data, whichmanip, movedown_distance ):
    rospy.loginfo( " "  )
    rospy.loginfo( "--"  )
    rospy.loginfo( "Putting down on table"  )

    #compute how much we need to move down based on the distance between the palm and the table surface
    ## DO NOT DELETE, FOLLOWING CODE IS FOR TEST AS OF SEP 1ST, 2011
    ## if movedown_distance < 0:
    ##     try:
    ##         palm_above_table_distance = getattr( global_data.grasp_parameter, 'place_on_distance')
    ##     except:
    ##         rospy.logwarn( 'No grasp_parameter for movedown_distance in global_data. Using default.' )
    ##         palm_above_table_distance = 0.16

    ## manips = global_data.robot.GetManipulators()
    ## T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
    ## xa_palm, ya_palm, za_palm, t_palm = MatrixToAxes( TSRUtil.E2P(T0_ee_curr) )
    
    ## ok, tablePose = GetObjectPose( global_data, "darpatable" )
    ## xa_table, ya_table, za_table, t_table = MatrixToAxes( PoseToTransform( tablePose ) )
    
    ## pushdir = za_table
    ## movedown_distance = palm_above_table_distance - (t_palm[2] - t_table[2])

    # get table pose
    gotTablePose, tablePose = GetObjectPose( global_data, "darpatable" )
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
    pushdir = -za_t

    breakOnCollision = 1
    block = True
    #success, reason, dist = MoveStraight( whichmanip, global_data, pushdir,
    #    movedown_distance, breakOnCollision, block )
    success, FreeChainList = GuardedApproachAlongPalmService(whichmanip, global_data, movedown_distance, pushdir )
    time.sleep(3.0)
    success, status = UpdateRobot( global_data, whichmanip )
    return success, "UpdateRobot"


'''
Open hand and release object
SetExtraMass to zero 
Then optionally backoff (Note if backoff is +ve, the hand will move *back*)
Optionally also delete object being released 
'''
def ReleaseAndBackOff( global_data, whichmanip, backoff, deleteFlag=False, openToExtent = 0 ):
    # Release objects 
    releasedTargets = ''
    grabbedobjs = global_data.robot.GetGrabbed()
    if grabbedobjs:
        for obj in grabbedobjs:
            if global_data.robot.GetManipulators()[whichmanip].IsGrabbing(obj):
                global_data.robot.Release(obj)
                releasedTargets = releasedTargets + ' %s' % obj.GetName() 

    # Set extra mass to zero (no need to check for objects)
    ResetExtraMass( global_data, whichmanip )
    HandPositionOpen(True, global_data, openToExtent)
    time.sleep(2.0)
    UpdateRobot(global_data, whichmanip)

    
    #reset the arm speed
    SetArmSpeedFast()
    
    # back off 
    if backoff > 0:
        rospy.loginfo( "Moving *back* %.3fm..." % backoff )
        MoveStraightForward( global_data, whichmanip, -backoff, True )
    success, reason, positions =  WaitForHand(whichmanip)
    if not IsHandOpened( positions ):
        return 0, "Hand Failed to open.  Finger positions: %s" %(positions)
        
    if deleteFlag:
        RemoveObjects( global_data, releasedTargets )

    return 1,"Done"

def ApplyForceInPalmDir( global_data, whichmanip, palm_force_dir, force_magnitude, relaxation = 0.0 ):
    """
    Run the apply force service.  DOES NOT BLOCK
    @param palm_force_dir - The direction of the applied force in end effector coordinates
    @param force_magnitude - the magnitude of the desired force (Newtons)
    @relaxation - fraction by which to reduce the joint torques commanded to
    reduce errors in the angle of the end effector - makes command softer
    """
    
    
    robot = global_data.robot
    #Get the Z direction of the palm    
    manip = robot.GetManipulators()[whichmanip]
    T0_ee = mat(manip.GetEndEffectorTransform())
    world_dir = dot(T0_ee[:3,:3],palm_force_dir)
    Wam0_dir = WorldToWam0(world_dir)
    
    success, reason = ApplyForceSrv( Wam0_dir, force_magnitude, relaxation )
    if success > 0:
        reason = "ApplyForce: succeeded"
        rospy.loginfo( '  ApplyForce: succeeded' );
    else:
        rospy.loginfo( '  ApplyForce: failed (%s)' % reason );
        reason = "ApplyForce: fail"
    return success, reason


def CenteredFingerClose(global_data, whichmanip, final_dofs = array([1.6,1.6,1.6,0])):
    """
    Close the hand in apply force mode with the compliant direction in the x axis of the hand
    which is perpendicular to finger closing.  This allows the hand to center on the object
    when one of the fingers hits it before the others

    @param final_dofs - desired final hand configuration as a 4 element numpy array
    """
    #now tare force and close the fingers very gently and try to center the palm w.r.t the drill horizontally
    palm_force_dir = array([1,0,0])
    force_magnitude = 0
    rospy.loginfo(" Taring the F/T ")
    TareForceTorque()
    for i in range(3):
        success, reason = ApplyForceInPalmDir( global_data, whichmanip, palm_force_dir, force_magnitude )
        if success:
            break
        TareForceTorque()
        time.sleep(1)
        
    if not success:
        rospy.loginfo( " ApplyForceInPalmDir is not successful (%s)" % reason)
        return success, "CenteredFingerClose: ApplyForceInPalmDir failed %s"%reason

    SetHandSpeedScalar( 0.5 )
    HandPositionMove(final_dofs)
    success, reason = WaitForHand ( whichmanip, timeout=15.0 )
    StopForceSrv()
    SetHandSpeedScalar( 1.5 )

    return success, reason

'''
This is a helper function that starts the apply force service according to the
force magnitude and force direction passed in
But be aware of the necessity that this function does not stop the force service
and the caller is required to turn it off when necessary
'''
def startApplyForce(forcedir, force, frame='WORLD'):

    ## #DO NOT DELETE, THE FOLLOWING CODE IS FOR TEST AS OF SEP 1ST, 2011
    ## if frame == 'PALM':
    ##     manip = robot.GetManipulators()[whichmanip]
    ##     T0_ee = mat(manip.GetEndEffectorTransform())
    ##     world_dir = dot(T0_ee[:3,:3],forcedir)
    ## elif frame == 'WORLD':
    ##     world_dir = forcedir
    ## wamdir = WorldToWam0(world_dir)
    
    wamdir = WorldToWam0(forcedir)
    rospy.loginfo('About to start applying force along world direction: [%s], which is transferred to wam direction: [%s]' % (forcedir, wamdir))
    # tare force for apply force service
    for r in range( 0, 3 ):
        rospy.loginfo( "Taring F/T" )
        TareForceTorque()
        time.sleep( 0.5 )
        
    # tare force should be finished up to now
    # call apply force service, try 5 times
    success = 0
    attempts = 0
    while success == 0 and attempts < 5:
        attempts += 1
        rospy.loginfo( "Applying downward force" )
        success, reason = ApplyForceSrv( wamdir, force )
        if success > 0:
            rospy.loginfo( '  ApplyForce: succeeded' );
            break
        else:
            rospy.loginfo( '  ApplyForce: failed (%s)' % reason );
        time.sleep( 1 )

    return success, reason


def liftUp(global_data, whichmanip, targetName):
    rospy.loginfo( "Lifting up object so that the EE is enough far above the table" )
    #Get the EE transform
    #pdb.set_trace()
    manips = global_data.robot.GetManipulators()
    T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
    xa_palm, ya_palm, za_palm, t_palm = MatrixToAxes( TSRUtil.E2P(T0_ee_curr) )
    
    gotTablePose, tablePose = GetObjectPose( global_data, "darpatable" )
    xa_table, ya_table, za_table, t_table = MatrixToAxes( PoseToTransform( tablePose ) )
    
    liftdir = za_table
    idealDist = getattr(global_data.grasp_parameter, 'lift_up_distance')
    liftdist = idealDist - (t_palm[2] - t_table[2])

    if 'darpapelican' in targetName:
       liftdist = 0.05

    # Temporarily disable grabbed objects from collision checking for initial lift
    for body in global_data.robot.GetGrabbed():
       print 'Disabling', body
       body.Enable(False)
    success, reason, dist = MovePersistent( global_data, whichmanip, liftdir, liftdist, True, cbirrt=True )
    for body in global_data.robot.GetGrabbed():
       body.Enable(True)
    if not success:
       print 'Failed to lift up!'
       return success, reason, dist

    if 'darpapelican' in targetName:
       print 'Doing pelican case specific lift'
       time.sleep(3)
       # Eventually, we should update robot
       T0_ee_curr = mat(manips[whichmanip].GetEndEffectorTransform())
       xa_table, ya_table, za_table, t_table = MatrixToAxes( PoseToTransform( tablePose ) )
       # Goal TSRs above current palm location, oriented downwards
       # Solve for palm location (w frame) w.r.t. world frame (same as table orentation, farther upwards)
       T_world_table = array(PoseToTransform(tablePose))
       R_table_palm = array(
          [[ 1., 0., 0.],
           [ 0.,-1., 0.],
           [ 0., 0.,-1.]])
       T_world_palm = eye(4)
       T_world_palm[0:3,0:3] = dot(T_world_table[0:3,0:3], R_table_palm)
       # Set location of palm frame
       T_world_palm[0:3,3] = t_palm
       #### NOTE: HARD CODED LIFT TO 0.4m IN Z DIRECTION ABOVE TABLE LOCATION
       T_world_palm[2,3] = t_table[2] + 0.55

       print '>>> Current palm pose:'
       print array(TSRUtil.E2P(mat(manips[whichmanip].GetEndEffectorTransform())))
       print '>>> Desired palm pose:'
       print T_world_palm

       # Get T_w_ee
       T_w_ee = array(TSRUtil.__Tp_e)

       # Get Bw
       #Bw = array([-0.10,0.10, -0.10,0.10, 0.,0., -0.2,0.2, -0.2,0.2, -pi,pi])
       Bw = array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

       #p = TSRPose.TSRPose(T0_w_in=mat(T_world_palm), Tw_e_in=mat(T_w_ee), Bw_in=mat(Bw))
       #tsr_string = TSRListToTSRString( global_data, whichmanip, [p], [])
       #print 'TSR String:', tsr_string
       TSRchains = ''
       # Add goal TSR
       TSRstring = TSR.SerializeTSR(global_data.robot.GetActiveManipulatorIndex(), 'NULL', mat(T_world_palm), mat(T_w_ee), mat(Bw))
       TSRchains += TSR.SerializeTSRChain(0,1,0,1,TSRstring,'NULL',array([]))
       # Add whole-trajectory TSR
       Bw = array([-0.10,0.10, -0.10,0.10, -1.0,1.0, -1.7,1.7, -1.7,1.7, -pi,pi])
       TSRstring = TSR.SerializeTSR(global_data.robot.GetActiveManipulatorIndex(), 'NULL', mat(T_world_palm), mat(T_w_ee), mat(Bw))
       #TSRchains += TSR.SerializeTSRChain(0,0,1,1,TSRstring,'NULL',array([]))

       cmd = 'RunCBiRRT smoothingitrs 250 psample 0.1 ' + TSRchains
       #for body in global_data.robot.GetGrabbed():
       #   print 'Disabling', body
       #   body.Enable(False)
       resp = global_data.probs_cbirrt.SendCommand(cmd)
       #for body in global_data.robot.GetGrabbed():
       #   body.Enable(True)
       if int(resp) != 1:
          print 'Planner failed'
          raw_input('Press enter to try with pelican case collisions turned off ...')
          pdb.set_trace()
          for body in global_data.robot.GetGrabbed():
             print 'Disabling', body
             body.Enable(False)
          resp = global_data.probs_cbirrt.SendCommand(cmd)
          for body in global_data.robot.GetGrabbed():
             body.Enable(True)
          if int(resp) != 1:
             return False, 'cbirrt planner failed', 0.0
       
       # run trajectory
       global_data.orEnv.LockPhysics(False)
       rospy.logdebug('running trajectory')
       RunBlendedTrajectory( global_data, 0, 'cmovetraj.txt', whichmanip )
       rospy.logdebug("trajectory executed")
       if True: # definitely block
           # the caller is responsible for relocking the physics if they are not waiting
           success, reason = WaitForRobot(global_data, 1)
           if not success:
               rospy.logwarn("robot not done")
               global_data.orEnv.LockPhysics(True)
               return 0,"Robot not done:%s .  Moved %d"%(reason, distMoved), distMoved
           global_data.orEnv.LockPhysics(True)
       rospy.logdebug("returning")
       
    return success, reason, dist

