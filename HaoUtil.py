from trajectory_planner import *
from numpy import *

####################### Calibration Utilities ####################

PALM_IN_EE = array([[cos(20*pi/180), -sin(20*pi/180), 0, 0],
                    [sin(20*pi/180), cos(20*pi/180), 0, 0],
                    [0,0,1,145.158],#this is approximate
                    [0,0,0,1]])

FT_IN_EE = array([[0, -1, 0, 0],
                  [1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

FT_IN_PALM = dot( linalg.inv(PALM_IN_EE), FT_IN_EE )

######################## Math Utilities ##################
def GetTransfFromAxisAngle(axis, angle):
    '''
    Compute the transform matrix for the rotation
    around axis by angle
    '''
    sa = sin(angle/2)
    ca = cos(angle/2)
    q = array([axis[0] * sa, #qx
               axis[1] * sa, #qy
               axis[2] * sa, #qz
               ca]) #qw
    q_normalized = q/linalg.norm(q)
    p = array([0,0,0])

    m = GetTransfFromQuaternionPosition(q_normalized,p)
    return m

def GetTransfFromQuaternionPosition(q, p):
    '''
    synthesize the transform matrix from quaternion q, and position p
    q is [qx, qy, qz, qw]
    '''
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    m = array([[ 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, p[0]],
               [ 2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw, p[1]],
               [ 2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy, p[2]],
               [ 0,0,0,1]])
    return m

def GetRotationAngle(tf1, tf2):
    '''
    Calculate the exact rotation between the two tf
    return the rotation axis and rotation angle
    '''
    rot = dot( linalg.inv(tf1), tf2 )
    inner_product = ( rot[0,0] + rot[1,1] + rot[2,2] - 1 )/2
    if inner_product > 0.99999:
        inner_product = 0.99999
    elif inner_product < -0.99999:
        inner_product = -0.99999
    angle = arccos(inner_product)
    return angle

    
######################## FT Utilities ####################
wrenchMsg = None
wrenchTracker = None

def WrenchTrackingCallback(wrench_msg):
    global wrenchMsg
    wrenchMsg = wrench_msg

def StartWrenchTracking():
    global wrenchTracker
    wrenchTracker = rospy.Subscriber("/ForceTorque/Readings", geometry_msgs.msg.Wrench, WrenchTrackingCallback)

def StopWrenchTracking():
    global wrenchTracker
    global wrenchMsg
    wrenchTracker.unregister()
    wrenchTracker = None
    wrenchMsg = None

######################## Hand related ##########################
velocity = array([0.2,0.2,0.2,0])
old_hand_position = array([-3,-3,-3,-3])
def GuardedCloseHand(global_data, speed = 0.2):
    global velocity
    velocity = array([speed, speed, speed,0])
    set_handstate_callback(global_data, GuardedFingerHitCallback)
    move_hand_velocity(velocity, False)
    WaitForHand(30)
    rospy.loginfo("done closing...")
    unset_handstate_callback(global_data)

def WaitForHand(timeout):
    global velocity
    iteration = 0
    num_iteration = timeout / 0.2
    stopped = False
    rospy.loginfo("iter: %s", num_iteration)
    global old_hand_position
    epsilon = 0.000001
    while (not stopped) and (iteration < num_iteration):
        if velocity[0] < 0.1 and velocity[1] < 0.1 and velocity[2] < 0.1:
            stopped = True
        """test how much the finger has travelled"""
        position = array(get_barrett_joints())
        travelled = position - old_hand_position
        old_hand_position = position
        if abs(travelled[0]) < epsilon and abs(travelled[1]) < epsilon and abs(travelled[2]) < epsilon:
            rospy.loginfo("stopped")
            stopped = True
        iteration = iteration + 1
        sleep(0.2)
        rospy.loginfo("is: %s", iteration)

def GuardedFingerHitCallback(handstate_msg):
    #rospy.loginfo("calling: %s", handstate_msg)
    strain = handstate_msg.strain
    position = array(handstate_msg.positions)
    #rospy.loginfo("callign: %s", strain)
    global velocity
    epsilon = 0.000001
    change = False
    threshold = 100
    if (strain[0] > 4*threshold) and velocity[0] > epsilon:
        velocity[0] = 0.0
        rospy.loginfo("1 done")
        change = True
    if (strain[1] > 4*threshold) and velocity[1] > epsilon:
        velocity[1] = 0.0
        rospy.loginfo("2 done")
        change = True
    if (strain[2] > threshold) and velocity[2] > epsilon:
        velocity[2] = 0.0
        rospy.loginfo("3 done")
        change = True
    #rospy.loginfo("new velocity: %s", velocity)
    if change:
        move_hand_velocity(velocity, False)
        


######################## Motion related ##########################
guarded_on_force = True

delay_num_local = 5
hits_local = delay_num_local
ft_threshold = 14
ft_direction = array([0,0,-1])

delay_miss = 5
miss = delay_miss
move_until_miss_threshold = 50
move_until_miss_direction = array([0,0,-1])
isMissed = False
isHit = False

def RotateHand(global_data, axis, frame = 'PALM', angle = 5*pi/180, fast = False):
    cancel_arm_motion()
    current_arm_tran = get_staubli_cartesian_as_tran()
    if frame == 'PALM':
        axisInPalm = axis
        axisInEE = dot( PALM_IN_EE[0:3,0:3], axisInPalm )
        offset_tran = GetTransfFromAxisAngle(axisInEE, angle)
    elif frame == 'WORLD':
        axisInWorld = axis
        WORLD_IN_EE = linalg.inv(current_arm_tran)
        axisInEE = dot( WORLD_IN_EE[0:3,0:3], axisInWorld )
        offset_tran = GetTransfFromAxisAngle(axisInEE, angle)
    ee_tran = dot(current_arm_tran, offset_tran)
    """So far, this is only for the shake test"""
    if fast:
        p = [2,2,2]
    else:
        p = []
    send_cartesian_goal(ee_tran, False, p)
    #wait for as long as 60 seconds
    WaitForArm(60)
    
def GuardedRotateUntilHit(global_data, axis, frame = 'PALM', angle = 5*pi/180, torque = 100):
    '''
    Guarded version of RotateHand
    We want to use it to re-orient the palm by tracking the torque
    '''
    global ft_direction
    global ft_threshold
    global guarded_on_force
    global isHit

    #setup global variable for monitoring
    guarded_on_force = True
    isHit = False
    ft_threshold = torque
    if angle < 0:
        angle = - angle
        angle = - angle
    ft_direction = -axis

    #prepare for the motion
    initial_arm_tran = get_staubli_cartesian_as_tran()

    #start motion
    SetFTCallback(global_data, FTGuardedHitCallback)
    RotateHand(global_data, axis, frame, angle)
    current_arm_tran = get_staubli_cartesian_as_tran()
    angle_moved = GetRotationAngle(initial_arm_tran, current_arm_tran)
    UnsetFTCallback(global_data)
    return angle_moved, isHit
    
    
def GuardedRotateUntilMiss(global_data, axis, frame = 'PALM', angle = 5*pi/180, torque = 100):
    '''
    Guarded version of RotateHand
    We want to use it to re-orient the palm by tracking the torque
    '''
    global ft_direction
    global ft_threshold
    global guarded_on_force
    global move_until_miss_threshold
    global move_until_miss_direction
    global isHit
    global isMissed

    #setup global variables
    isHit = False
    isMissed = False
    guarded_on_force = False
    ft_threshold = torque
    if angle < 0:
        angle = - angle
        axis = - axis
    ft_direction = -axis
    move_until_miss_threshold = 10
    move_until_miss_direction = axis

    #prepare for the motion
    initial_arm_tran = get_staubli_cartesian_as_tran()

    #start the motion
    SetFTCallback(global_data, FTGuardedMissCallback)
    RotateHand(global_data, axis, frame, angle)
    current_arm_tran = get_staubli_cartesian_as_tran()
    angle_moved = GetRotationAngle(initial_arm_tran, current_arm_tran)
    UnsetFTCallback(global_data)
    return angle_moved, isMissed, isHit
    
    
def MoveHand(global_data, direction, frame = 'PALM', distance = 0.1):
    '''
    move the hand in the hand corordinate system
    '''
    cancel_arm_motion()
    current_arm_tran = get_staubli_cartesian_as_tran()
    directionInPalm = direction
    directionInEE = dot( PALM_IN_EE[0:3,0:3], directionInPalm )
    offset_tran = eye(4)
    offset_tran[0:3,3] = directionInEE * distance
    ee_tran = dot(current_arm_tran, offset_tran)
    #rospy.loginfo('from %s \nto \n%s', current_arm_tran, ee_tran)
    send_cartesian_goal(ee_tran, False)
    #wait for as long as 60 seconds
    WaitForArm(60)

def GuardedMoveUntilHit(global_data, direction, frame = 'PALM', distance = 0.1, force = 10.0, tare_force = True):
    '''
    Guarded version of MoveHand
    '''
    global ft_direction
    global ft_threshold
    global guarded_on_force
    global isHit

    #setup global variable for monitoring
    guarded_on_force = True
    isHit = False
    ft_threshold = force
    if distance < 0:
        distance = - distance
        direction = - direction
    ft_direction = -direction

    #prepare for the motion
    initial_arm_tran = get_staubli_cartesian_as_tran()
    if tare_force:
        tare_force_torque()

    SetFTCallback(global_data, FTGuardedHitCallback)
    #start motion
    MoveHand(global_data, direction, frame, distance)

    #calculate motion result
    current_arm_tran = get_staubli_cartesian_as_tran()
    distance_moved = linalg.norm(initial_arm_tran[0:3,3] - current_arm_tran[0:3,3])
    UnsetFTCallback(global_data)
    return distance_moved, isHit

def GuardedMoveUntilMiss(global_data, direction, miss_direction, frame = 'PALM', distance = 0.1, stop_force = 10.0, miss_force = 100.0, tare_force = False):
    '''
    move along direction and monitor the force/torque input
    stop when the force is not detected along miss_direction
    '''
    global ft_direction
    global ft_threshold
    global move_until_miss_threshold
    global move_until_miss_direction
    global isHit
    global isMissed
    global guarded_on_force
        
    #setup locally global variables 
    if distance < 0:
        distance = - distance
        direction = - direction
    ft_direction = -direction
    ft_threshold = stop_force
    move_until_miss_threshold = miss_force
    move_until_miss_direction = miss_direction
    guarded_on_force = True
    isHit = False
    isMissed = False

    #prepare for the motion
    initial_arm_tran = get_staubli_cartesian_as_tran()
    if tare_force:
        tare_force_torque()

    #start the motion
    SetFTCallback(global_data, FTGuardedMissCallback)
    MoveHand(global_data, direction, frame, distance)
    current_arm_tran = get_staubli_cartesian_as_tran()
    distance_moved = linalg.norm(initial_arm_tran[0:3,3] - current_arm_tran[0:3,3])
    UnsetFTCallback(global_data)
    return distance_moved, isMissed, isHit


def FTGuardedHitCallback(wrench_msg):
    global hits_local
    global delay_num_local
    global ft_threshold
    global ft_direction
    global isHit
    global guarded_on_force

    if guarded_on_force:
        forceInFT = array( [wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z] )
        forceInPalm = dot( FT_IN_PALM[0:3,0:3], forceInFT )
        ftProjected = dot( forceInPalm, ft_direction )
    else:
        torqueInFT = array( [wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z] )
        torqueInPalm = dot( FT_IN_PALM[0:3,0:3], torqueInFT )
        ftProjected = dot( torqueInPalm, ft_direction )

    #record when the force exceeds the threshold
    if ftProjected > ft_threshold:
        #rospy.loginfo('forceInFT: %s\n forceInPalm: %s\n ftProjected: %s\n threshold: %s', forceInFT, forceInPalm, ftProjected, ft_threshold)
        hits_local -= 1 
    else:
        hits_local = delay_num_local
    if hits_local == 0:
        isHit = True
        cancel_arm_motion()
        rospy.loginfo('hit')
    return

def FTGuardedMissCallback(wrench_msg):
    global hits_local
    global delay_num_local
    global ft_threshold
    global ft_direction
    global guarded_on_force
    global isHit

    if guarded_on_force:
        forceInFT = array( [wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z] )
        forceInPalm = dot( FT_IN_PALM[0:3,0:3], forceInFT )
        ftProjected = dot( forceInPalm, ft_direction )
    else:
        torqueInFT = array( [wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z] )
        torqueInPalm = dot( FT_IN_PALM[0:3,0:3], torqueInFT )
        ftProjected = dot( torqueInPalm, ft_direction )

    #record when the force exceeds the threshold
    if ftProjected > ft_threshold:
        #rospy.loginfo('forceInFT: %s\n forceInPalm: %s\n ftProjected: %s\n threshold: %s', forceInFT, forceInPalm, ftProjected, ft_threshold)
        hits_local -= 1 
    else:
        hits_local = delay_num_local
    if hits_local == 0:
        rospy.logerr('hit')
        isHit = True
        cancel_arm_motion()
        return

    #check if we missed any contact
    global miss
    global delay_miss
    global move_until_miss_threshold
    global move_until_miss_direction
    global isMissed

    if guarded_on_force:
        ftProjected = dot( forceInPalm, move_until_miss_direction )
    else:
        ftProjected = dot( torqueInPalm, move_until_miss_direction )

    if ftProjected < move_until_miss_threshold:
        #rospy.loginfo('forceInFT: %s\n forceInPalm: %s\n ftProjected: %s\n threshold: %s', forceInFT, forceInPalm, ftProjected, ft_threshold)
        miss -= 1 
    else:
        miss = delay_miss
    if miss == 0:
        rospy.logerr('missed')
        isMissed = True
        cancel_arm_motion()
        return
    
    return

def WaitForArm(timeout):
    '''
    monitor the arm motion and returns right when the arm is detected stopped
    '''
    stop_times = 0
    stopped = False
    iteration_max = timeout / 0.2
    iteration = 0
    initial_arm_tran = get_staubli_cartesian_as_tran()

    while (not stopped) and (iteration < iteration_max):
        iteration += 1
        sleep(0.2)
        current_arm_tran = get_staubli_cartesian_as_tran()
        distance = linalg.norm(initial_arm_tran[0:3,3] - current_arm_tran[0:3,3])
        angle = GetRotationAngle(initial_arm_tran, current_arm_tran)

        if distance < 0.0005 and angle < 1 * pi / 180:
            stop_times += 1
            #rospy.loginfo('stop_times: %d', stop_times)
        else:
            stop_times = 0
            initial_arm_tran = current_arm_tran
            #rospy.loginfo('still moving: %f, %f', distance, angle)

        stopped = (stop_times > 5)
        if not stopped:
            #rospy.loginfo('still moving:%s', distance)
            pass
    #rospy.loginfo('arm stopped')

def ReOrient(global_data,level):
    '''
    reorient the palm to align with a surface using rotate until miss
    tested but not good, should use ReOrient2 instead
    '''
    global wrenchMsg
    StartWrenchTracking()
    angle = level*pi/180
    dist = GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.1, 10)

    for i in range(5):
        pdb.set_trace()
        #move to establish contacts
        if not i == 0:
            dist = GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.1, 10, False)
        #analyze the torque to decide re-orientation direction
        forceInFT = array( [wrenchMsg.force.x, wrenchMsg.force.y, wrenchMsg.force.z] )
        torqueInFT = array( [wrenchMsg.torque.x, wrenchMsg.torque.y, wrenchMsg.torque.z] )
        torqueInPalm = dot( FT_IN_PALM[0:3,0:3], torqueInFT )
        axisInPalm = torqueInPalm / linalg.norm(torqueInPalm)

        angle_moved, isMissed, isHit = GuardedRotateUntilMiss(global_data, axisInPalm, 'PALM', angle)
        if angle_moved < 2*pi/180 and isHit:
            rospy.loginfo( 'less than 2 degrees moved: %f', angle_moved * 180 / pi)
        
        angle = angle * 0.8

        rospy.loginfo('torque/force ratio: %f ', linalg.norm(torqueInFT) / linalg.norm(forceInFT))

        
def ReOrient2(global_data, angle):
    '''
    reorient the palm to align with a surface
    this is a better version than ReOrient
    '''
    global wrenchMsg
    StartWrenchTracking()

    for i in range(5):
        #pdb.set_trace()
        #move to establish contacts
        dist = GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.1, 10)
        #analyze the torque to decide re-orientation direction
        forceInFT = array( [wrenchMsg.force.x, wrenchMsg.force.y, wrenchMsg.force.z] )
        torqueInFT = array( [wrenchMsg.torque.x, wrenchMsg.torque.y, wrenchMsg.torque.z] )
        torqueInPalm = dot( FT_IN_PALM[0:3,0:3], torqueInFT )
        axisInPalm = torqueInPalm / linalg.norm(torqueInPalm)

        #move back a little bit to get away from friction
        MoveHand(global_data, array([0,0,-1]), 'PALM', 0.001)
        #rotate away from collision all the way to the other side
        #pdb.set_trace()
        angle_moved, isHit = GuardedRotateUntilHit(global_data, axisInPalm, 'PALM', angle)
        if angle_moved < 2*pi/180 and isHit:
            rospy.loginfo( 'less than 2 degrees moved: %f', angle_moved * 180 / pi)
            return

        #move back to the middle rotation angle
        angle_moved, isHit = GuardedRotateUntilHit(global_data, -axisInPalm, 'PALM', angle_moved/2)

        rospy.loginfo('torque/force ratio: %f ', linalg.norm(torqueInFT) / linalg.norm(forceInFT))

    StopWrenchTracking()
        
def Reorient3(global_data, angle, axis_in_world):
    '''
    reorient the palm so that the y axis is aligned with axis_in_world
    still not sure of the necessity of this function
    need re-consideration
    '''
    #move back 0.2m
    MoveHand(global_data, array([0,0,-1]), 'PALM', 0.2)

    #rotate -90 degrees around axis_in_wolrd
    #if it is aligned well, the fingers should be perpendicular to the surface
    RotateHand(global_data, axis_in_world, 'WORLD', -0.5*pi)

    for i in range(5):
        #move down to the surface again
        dist = GuardedMoveUntilHit(global_data, array([1,0,0]), 'PALM', 0.3, 10)
        #analyze the torque to decide re-orientation direction
        forceInFT = array( [wrenchMsg.force.x, wrenchMsg.force.y, wrenchMsg.force.z] )
        torqueInFT = array( [wrenchMsg.torque.x, wrenchMsg.torque.y, wrenchMsg.torque.z] )
        torqueInPalm = dot( FT_IN_PALM[0:3,0:3], torqueInFT )
        axisInPalm = torqueInPalm / linalg.norm(torqueInPalm)
        if axisInPalm[2] > 0:
            axis = array([0,0,1])
        else:
            axis = array([0,0,-1])
        angle_moved, isHit = GuardedRotateUntilHit(global_data, axis, 'PALM', angle)

        rospy.loginfo('torque/force ratio: %f ', linalg.norm(torqueInFT) / linalg.norm(forceInFT))

def go_home_r(global_data):
    done = False
    while(not done):
        done = go_home(global_data)
    
