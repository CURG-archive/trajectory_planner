import roslib 
roslib.load_manifest("staubliTX60")
import rospy
from ft_manager import *
import staubliTX60
from staubliTX60.srv import *
import staubliTX60.msg
import geometry_msgs.msg
import tf
import tf.transformations
import tf_conversions.posemath as pm
from numpy import *
import actionlib
import copy as cp
import actionlib_msgs.msg

_delay_num = 5
_hits = _delay_num
_threshold = -14


def point_list_to_msg_list( dof_traj_list ):
    """@brief - Structure a list of dof trajectory points as a ros message along with appropriate
                staubli controller parameters
       @param dof_traj_list - list of dof trajectory points

       Returns list of ros joint trajectory messages.
    """
    p = staubliTX60.msg.StaubliMovementParams(jointVelocity = 0.4, jointAcc = 0.04, jointDec= 0.04,
                                              endEffectorMaxTranslationVel =  9999.0,
                                              endEffectorMaxRotationalVel =  9999.0,
                                              movementType = 1,
                                              distBlendPrev = 0.01,
                                              distBlendNext = 0.01)

    joint_point_msg_list = []
    for point in dof_traj_list:
        pf = [float(p2) for p2 in point]
        joint_point_msg_list.append(staubliTX60.msg.JointTrajectoryPoint(jointValues = cp.deepcopy(pf), params = cp.deepcopy(p)))
    joint_point_msg_list[-1].params.distBlendPrev = 0
    joint_point_msg_list[-1].params.distBlendNext = 0
    joint_point_msg_list[-1].params.movementType = 0
    
    return joint_point_msg_list
        

def run_staubli_on_joint_goal(j, blocking = False):
    """@brief - Move staubli to a specified joint position
       @param j - Desired joint position
       @param blocking - Whether to block on completion of action before returning.
       
       Returns whether motion was successful if blocking or successfully enqueued if not blocking
    """
    client = actionlib.SimpleActionClient('setJoints', staubliTX60.msg.SetJointsAction) 
    if not client.wait_for_server():
        print "SetJointTrajectory action server could not be found "
        return False, []
    action_goal = staubliTX60.msg.SetJointsGoal(j = j)
    client.send_goal(action_goal)
    if blocking:
        client.wait_for_result()
    return client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED or client.get_state() == actionlib_msgs.msg.GoalStatus.PENDING or client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE, client, j


#def fix_traj_list(dof_traj_list):
#    return [[j%(2*pi) for j in traj_point] for traj_point in dof_traj_list]] 



def run_staubli_on_trajectory_point_message( dof_traj_list , blocking = False ):
    """@brief - take a list of trajectory point messages and run the staubli
       @param traj_message_list - set of trajectory point messages
       @param blocking - should we run the robot to finish trajectory
    
       Returns whether motion was successful if blocking or successfully enqueued if not blocking.
    """
        
    traj_message_list = point_list_to_msg_list( dof_traj_list )

    client = actionlib.SimpleActionClient('setJointTrajectory', staubliTX60.msg.SetJointTrajectoryAction) 
    if not client.wait_for_server():
        print "SetJointTrajectory action server could not be found "
        return False, client
    
    action_goal = staubliTX60.msg.SetJointTrajectoryGoal(jointTrajectory = traj_message_list)
    client.send_goal(action_goal)
    
    if blocking:
        client.wait_for_result()

    return client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED or client.get_state() == actionlib_msgs.msg.GoalStatus.PENDING or client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE, client


def get_staubli_joints():
    """@brief - Read the staubli's joints.

       Returns joint vector on success, empty list on failure. 
    """
    
    rospy.wait_for_service('getJoints')
    try:
        get_joints = rospy.ServiceProxy( 'getJoints', GetJoints )
        resp1 = get_joints()
        return resp1.j
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return []


def get_staubli_cartesian_as_pose_msg():
    """@brief - Read the staubli end effector's cartesian position as a pose message

       Returns pose message on success or empty message on failure
    """
    rospy.wait_for_service('getCartesian')
    try:
        get_cartesian = rospy.ServiceProxy( 'getCartesian', GetCartesian )
        resp1 = get_cartesian()
        # make srv x y z  rx ry rz euler angle representation into pose message
        pos = geometry_msgs.msg.Point( x = resp1.x, y = resp1.y, z = resp1.z )
        q = tf.transformations.quaternion_from_euler( resp1.rx , resp1.ry, resp1.rz ,'rxyz' )
        quat =  geometry_msgs.msg.Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] )
        return geometry_msgs.msg.Pose( position = pos, orientation = quat )
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return []
    
def get_staubli_cartesian_as_tran():
    """@brief - Read the staubli end effector's cartesian position as a 4x4 numpy matrix

       Returns 4x4 numpy message on success, empty message on failure
    """
    current_pose = get_staubli_cartesian_as_pose_msg()
    return pm.toMatrix(pm.fromMsg(current_pose))


def send_cartesian_goal(end_effector_tran, blocking = False, movement_params = []):
    """@brief - Send a cartesian goal to the robot. Return action client
       @param end_effector_tran - Desired end effector position
       @param blocking - Whether to block on completion of action
       @param movement_params - parameters for staubli arm motion. See staubli
                                documentation for details

       Returns action client and desired end effector transform.
    """
    
    if movement_params == []:
        p = staubliTX60.msg.StaubliMovementParams(jointVelocity = 0.4,
                                                  jointAcc = 0.04,
                                                  jointDec= 0.04,
                                                  endEffectorMaxTranslationVel =  9999.0,
                                                  endEffectorMaxRotationalVel =  9999.0,
                                                  movementType = 1,
                                                  distBlendPrev = 0.01,
                                                  distBlendNext = 0.01)
    else:
        p = staubliTX60.msg.StaubliMovementParams(jointVelocity = movement_params[0],
                                                  jointAcc = movement_params[1],
                                                  jointDec= movement_params[2],
                                                  endEffectorMaxTranslationVel =  9999.0,
                                                  endEffectorMaxRotationalVel =  9999.0,
                                                  movementType = 1,
                                                  distBlendPrev = 0.01,
                                                  distBlendNext = 0.01)
    
    client = actionlib.SimpleActionClient('setCartesian', staubliTX60.msg.SetCartesianAction) 
    if not client.wait_for_server():
        print "SetCartesian action server could not be found "
        return False, []

    action_goal = staubliTX60.msg.SetCartesianGoal()
    action_goal.x,action_goal.y,action_goal.z = end_effector_tran[0:3,3]
    action_goal.rx,action_goal.ry,action_goal.rz = tf.transformations.euler_from_matrix( end_effector_tran, 'rxyz' )
    action_goal.lineCtrl = 1
    action_goal.params = p
    client.send_goal(action_goal)
    
    if blocking:
        client.wait_for_result()
    return client, end_effector_tran


    

def cancel_arm_motion():
    """@brief - Cancel any active movement actions and stop arm. 
    
       Returns true if service call is successful, false otherwise. 
    """
    reset_motion = rospy.ServiceProxy('/cancelMotion', ResetMotion)
    try:
        reset_motion()
        print "arm motion cancelled"
        return 1
        rospy.logwarn("motion cancelled")
    except Exception as e:
        rospy.logwarn("cancelMotion failed %s"%e)
        return 0

def lift_arm(dist, blocking = False, movement_params = []):
    """@brief - Move arm in Z direction of world coordinate system.
       @param dist - Distance to move in meters.
       @param blocking - Whether to block on completion of action
       @param movement_params - parameters for staubli arm motion. See staubli
                                documentation for details

       Returns whether motion was successful if blocking or successfully enqueued if not blocking
    """
    current_arm_tran = get_staubli_cartesian_as_tran()
    offset_tran = eye(4)
    offset_tran[2,3] = dist
    ee_tran = dot(offset_tran, current_arm_tran)
    client, end_effector_tran = send_cartesian_goal(ee_tran, blocking, movement_params)
    return client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED or client.get_state() == actionlib_msgs.msg.GoalStatus.PENDING or client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE


    

def move_forward(dist, blocking = False, movementParams = []):
    """@brief - Move arm in Z direction of hand coordinate system.
       @param dist - Distance to move in meters.
       @param blocking - Whether to block on completion of action
       @param movement_params - parameters for staubli arm motion. See staubli
                                documentation for details

       Returns whether motion was successful if blocking or successfully enqueued if not blocking
    """
    current_arm_tran = get_staubli_cartesian_as_tran()
    offset_tran = eye(4)
    offset_tran[2,3] = dist
    ee_tran = dot(current_arm_tran, offset_tran)
    client, end_effector_tran = send_cartesian_goal(ee_tran, blocking, movementParams)
    return client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED or client.get_state() == actionlib_msgs.msg.GoalStatus.PENDING or client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE
    

def move_forward_to_contact2(dist, blocking = False, movementParams = []):
    """@brief - DEPRECATED: Move arm in Z direction of hand coordinate system by a particular distance or until contact is made.
       @param dist - Distance to move in meters.
       @param blocking - Whether to block on completion of action
       @param movement_params - parameters for staubli arm motion. See staubli
                                documentation for details

       Returns whether motion resulted in contacted.
    """
    tare_FT()
#    SetFTCallback(setFTData, FTThresholdCallback)
    move_forward(dist, blocking, movementParams)
    #sleep(10)
    return unset_ft_callback(global_data)
    

        
def SetFTCallback(global_data, callback_func):
    """@brief - DEPRECATED: enable force-torque sensor client      
       @param global_data - data structure for storing force-torque subscriber.
       @param callback_func - processes force-torque data

       
    """
    if not global_data.ft_sub == []:
        sub = global_data.ft_sub
        global_data.ft_sub = []
        sub.unregister()
        del sub
    global_data.ft_sub = rospy.Subscriber("/ForceTorque/Readings", geometry_msgs.msg.Wrench, callback_func)
    

def UnsetFTCallback(global_data):
    """@brief - DEPRECATED: disable force-torque sensor client.
       @param global_data - data structure for storing force-torque subscriber.
    """
    if not global_data.ft_sub == []:
        sub = global_data.ft_sub
        global_data.ft_sub = []
        sub.unregister()
        del sub
        return True
    return False


def tare_force_torque(blocking = True):
    """@brief - reset baseline for force-torque sensor
       @param blocking - block on return of service call

       Returns true of service call succeeds, false if service call fails.
    """
    global global_data
    if blocking:
        try:
            tare = rospy.ServiceProxy('/ForceTorque/TareService', EmptySrv)
            resp1 = tare()
            return resp1
        except:
            rospy.logwarn('Service Call to tare service failed')
            return 0
    else:
        if global_data.ft_tare_pub == []:
            global_data.ft_tare_pub = rospy.Publisher("/ForceTorque/Tare", EmptyMsg)
            global_data.ft_tare_pub.publish()
            sleep(4)
    return 1



def move_forward_to_contact(dist, blocking = True, wait_time = 10):
    """@brief - Move arm in Z direction of hand coordinate system by a particular distance or until contact is made.
       @param dist - Distance to move in meters.
       @param blocking - Whether to block on completion of action
       @param wait_time - Time in seconds to wait for completion

       Returns whether motion resulted in contacted.
    """
    tare_FT()
    ftw = FTWatcher(cancel_arm_motion)
    ftw.set_FT_callback()
    move_forward(dist, blocking)
    if not blocking:
        sleep(wait_time)
    return ftw.unset_FT_callback()


def lift_arm_to_contact(dist, blocking=True, wait_time = 10):
    """@brief - Move arm in Z direction of world coordinate system by a particular distance or until contact is made.
       @param dist - Distance to move in meters.
       @param blocking - Whether to block on completion of action
       @param wait_time - Time in seconds to wait for completion

       Returns whether motion resulted in contacted.
    """
    tare_FT()
    ftw = FTWatcher(cancel_arm_motion)
    ftw.set_FT_callback()
    lift_arm(dist, blocking)
    if not blocking:
        sleep(wait_time)
    return ftw.unset_FT_callback()
