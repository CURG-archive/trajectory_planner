"""@package trajectory_planning

Currently uses the openrave environment and only works with the Staubli arm. Could be made more agnostic to both the arm and the planner.

Helper functions for running a planner given only a set of start and end goals in robot coordinates

Currently uses cbirrt planner
"""
import roslib; roslib.load_manifest( "trajectory_planner" )
import rospy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs

import pdb
#import openravepy as orpy

import time
import math
from time import sleep

import tf
import tf.transformations
import tf_conversions.posemath as pm

import geometry_msgs
 
from tf import transformations as tr

from GraspUtil import *


import staubliTX60.msg
import staubliTX60.srv
import actionlib
from openravepy import *
import time
import copy as cp
import pr_msgs
from OWDUtil import *
import random
import numbers

#refactored ugly code that needs to die
from test_object_grasps import *
from staubli_manager import *
from staubli_or_interface import *
from barrett_manager import *

import sys

import warnings

random.seed(10)

#global global_data

class GlobalData:
    def __init__(self):
        self.or_env = []
        self.probs_manip = []
        self.listener = []
        self.ft_tare_pub = []
        self.ft_sub = []
        self.hs_sub = []

def stop_everything():
    """@brief - Disable all movement of robot.

    This function must be robust. Do not change it without testing. 
    """
    if not cancel_arm_motion() or not relax_barrett():
        rospy.logfatal('Stopping Failed. Dropping to debug. \n WARNING!!!! Will exit program before exiting function.')
        pdb.set_trace()
        sys.exit(-2)
    return 1

        


def handle_fatal(msg):
    """@brief - logs an error message and calls the exit function.
    Stops arm and hand. Sets hand torque to zero

    This function centralizes fatal error handling so that debugging
    facilities can be added to a single place in the future.

    One way to handle debugging is to add an exit handler. See atexit
    module for details. 
    
    @param msg - Error message to be logged.
    """
    rospy.logfatal(msg)
    stop_everything()    
    sys.exit(-1)
    


def pose_list_to_TSR_list(poses):
    """@brief - create a set of TSRs from pose message list
    @param - list of pose messages to convert to TSRs. See geometry_msgs.Pose
    for details. 
    """
    tsrList = []
    numGrasps = 10
    numGrasps = min( len( poses ), numGrasps )
    Bw_T = 0.01
    Bw_R = 2.5 * pi / 180
    for i in range( numGrasps ):
        T0_g = PoseToTransform( poses[i] )
        tsr = TSRUtil.ToTSRPose( T0_g, T0_g, TSRUtil.ToBw(Bw_T, Bw_R))
        tsrList.append( cp.deepcopy( tsr ) )
    return tsrList





def pose_list_to_tsr_string(pose_list, constrain_pose = [], start_sampling =0 , destination_sampling = 1):
    """@brief - Return a string representing a TSR

    @param pose_list - a list containing geometry_msgs.Pose
    @param constrain_pose - a boolean flag determining whether the final TSR attempts
    to maintain the starting wrist orientation during motion
    @param start_sampling - a boolean flag determining whether the TSR is used for start pose sampling
    @param destination_sampling - a boolean flag determining whether the TSR is used for goal pose sampling  
    """
    class fake_global_data:
        """@brief Fake global data structure for helper functions written for openraves manipulation applet
        
        """
        def __init__(self, psample):
            self.psample = psample

    fk = fake_global_data(100)
    pose_tsr_list = pose_list_to_TSR_list( pose_list )
    return TSRListToTSRString( fk, 0, pose_tsr_list , constrain_pose, len(pose_tsr_list), start_sampling, destination_sampling )




def dof_list_to_cbirrt_planning_string( joint_dof_list , is_start_flag ):
    """@brief - Return a cbirrt string for a DOF list
    @param joint_dof_list - a list of desired DOF positions
    @param is_start_flag - a boolean flag determining if this is a starting or goal dof set

    """
    planner_string = ""
    if is_start_flag:
        planner_string += "jointstarts "
    else:
        planner_string += "jointgoals "

    planner_string += "%d "%(len(joint_dof_list)) + " " + ' '.join('%.3f' % val for val in joint_dof_list)

    return planner_string
    
    

def cbirrt_planning_string( goal_list, starting_list = [], smoothing_iters = 100 ):
    """@brief - Return a string for planning a motion using a pose message list using CBirrt


    @param goal_list - a list containing geometry_msgs.Pose or a list of dofs
    @param starting_list - a list containing potential starting poses as geometry_msgs.Pose or a list of do
    
    This function sets up a default TSR around the given list of poses and returns a string
    containing appropriate poses.  This function assumes that if the starting pose list is empty
    the current robot pose in the openrave environment is the desired dofs

    both starting_dof_list and starting_pose_list being non-null is a logical inconsistency
    and is likely and error.  This will generate a warning.
    
    """

    planner_string = "RunCBiRRT "

    if starting_list:
        if isinstance(starting_list[0],numbers.Number):
            planner_string += dof_list_to_cbirrt_planning_string(starting_list, True)
        elif type(starting_list[0]) == type(geometry_msgs.msg.Pose()):
            pose_list_to_tsr_string( starting_list, constrain_pose = [], start_sampling =1 , destination_sampling = 0 )
        else:
            print "trajectory_planner::cbirrt_planning_string::Unknown starting list type"
            return False, [], []


    
    if goal_list == []:
        print "trajectory_planner::cbirrt_planning_string::Error no goal given"
        return False, [] ,[]
    else:
        if isinstance(goal_list[0],numbers.Number):
            planner_string += dof_list_to_cbirrt_planning_string(goal_list, False)
        elif type(goal_list[0]) == type(geometry_msgs.msg.Pose()):
            planner_string += pose_list_to_tsr_string( goal_list, constrain_pose = [], start_sampling =0 , destination_sampling = 1 )
        else:
            print "trajectory_planner::cbirrt_planning_string::Unknown goal list type"
            return False, [], []
            


    filename = "/home/armuser/ros/rosbuild_src/trajectory_planner/cmovetraj.txt"
    planner_string += " smoothingitrs %i filename %s"%(smoothing_iters, filename) + " \n"
    return True, planner_string, filename
             

    
def run_cbirrt_with_planner_string( or_env, planner_string ):
    """Run the cbirrt planner starting with a tsr string  -- specialized for the staubli arm
    @param or_env - openrave python environment
    @param tsr_string - tsr string
    return the success of the planner
    """
    print planner_string
    current_robot = check_or_load_staubli( or_env )
    current_robot.SetActiveDOFs(range(6))
    if not current_robot:
        print "trajectory_planner::run_cbirrt_with_tsr_string:: Failed to load staubli"
        return False

    p = RaveCreateProblem( or_env, 'CBiRRT' ) 
    or_env.LoadProblem( p, current_robot.GetName() )
    success = p.SendCommand( planner_string )
    return success
       
    

def run_cbirrt_with_pose_list ( or_env, goal_list, starting_list = [], smoothing_iters = 100 ):
    """Run the cbirrt planner starting with lists of poses or dof values
    @param or_env - openrave python environment
    @param goal_list - a list containing geometry_msgs.Pose or a list of dofs
    @param starting_list - a list containing potential starting poses as geometry_msgs.Pose or a list of dofs
    returns success and a filename of the trajectory
    """

    success, planner_string, filename = cbirrt_planning_string( goal_list, starting_list, smoothing_iters )
    success = run_cbirrt_with_planner_string( or_env, planner_string)

    if success:
            dof_list = dof_list_from_traj_file(filename, range(6))
            
            
        
    return success, filename, dof_list, goal_list




def is_home(global_data = []):
    """Helper function to determine if the magnitude of the difference between the current position
    and the home position is within some threshold.
    
    @param global_data - an optional parameter that can contain a home_position and home_threshold members that will
                         override the defaults.

                         
    The default home is [0,0,0,0,0,0], the default threshold is .1

    These defaults can also be overriden by setting the is_home.home_position and is_home.home_threshold
    attributes.

    global_data has precedence
    
    """

    """Set defaults for the function if necessary """

    

    if not hasattr(is_home, 'home_threshold'):
        is_home.home_threshold = .1

    home_threshold = is_home.home_threshold

    if not hasattr(is_home, 'home_position'):
        is_home.home_position = [0,0,0,0,0,0]

    home_position = is_home.home_position
    

    """Test for non-default inputs"""
    
    if global_data:
        if hasattr(global_data,'home_position'):
            home_position = global_data.home_position


        if hasattr(global_data,'home_threshold'):
            home_threshold = global_data.home_threshold
           
    j = array(get_staubli_joints())    

    return linalg.norm(j - home_position) < home_threshold

def go_home(global_data = [], robot_index = 0):
    """Make planned movement to the home position of the robot if possible, otherwise simply attempt to move the staubli
    using a joint command
    
    @param global_data - a parameter containing an openrave environment.
    @param robot_index - the index of the robot to be moved home in the openrave environment.
    
    home position default in function is [0,0,0,0,0,0]
    This can be overridden using by either setting the go_home.home_position attribute or passing in a global_data with
    a home_position parameter

    If making a joint motion with no planning, the function will pause with a warning unless you set its joint_motion_pause parameter
    to 0
    
    """


    """make sure the openrave environment is up to date."""
    

    if not hasattr(go_home, 'home_position'):
        go_home.home_position = [0,0,0,0,0,0]

    if not hasattr(go_home,'joint_motion_pause'):
        go_home.joint_motion_pause = 1

    home_position = go_home.home_position

    success = False
    """if we have a global_data, make a planned motion home """
    
    if global_data:
        if hasattr(global_data,'home_position'):
            home_position = global_data.home_position
        update_robot(global_data.or_env.GetRobots()[robot_index])
        success, dof_filename, dof_list, goal_list =  run_cbirrt_with_pose_list ( global_data.or_env, home_position, starting_list = [])        
        print dof_list
    else:
        """if we have not been given a valid global_data, simply make a joint motion        
        """
        rospy.logwarn("Going home using a joint motion.  NO TRAJECTORY COLLISION CHECKING")
        if go_home.joint_motion_pause:
            time.sleep(go_home.joint_motion_pause)
        
        dof_list = [[home_position]]

    """run the motion"""
        
    if success:
        success, client = run_staubli_on_trajectory_point_message( dof_list , True )
        
    return success

    


def run_cbirrt_with_tran ( or_env, tran, starting_list = [], smoothing_iters = 100 ):
    """Run the cbirrt planner starting with lists of poses or dof values
    @param or_env - openrave python environment
    @param goal_list - a list containing geometry_msgs.Pose or a list of dofs
    @param starting_list - a list containing potential starting poses as geometry_msgs.Pose or a list of dofs
    returns success, filename of the trajectory, discretized dof trajectory, and joints of final solution
    """
    
    j = or_env.GetRobots()[0].GetManipulators()[0].FindIKSolutions(tran, IkFilterOptions.CheckEnvCollisions)
    if j == []:
        warn("no ik solution for tran %s\n"%tran)
        return False, [], [],[]
    
    success = False
    planner_string = []
    filename = []
    dof_list = []
    for i in range(len(j)):
        success, planner_string, filename = cbirrt_planning_string( j[i].tolist(), starting_list, smoothing_iters )        
        success =  run_cbirrt_with_planner_string( or_env, planner_string)
        if success:
            dof_list = dof_list_from_traj_file(filename, range(6))
            dof_list.append(j[i].tolist())
            break
        
    return success, filename, dof_list, j
  


    
def convert_cartesian_world_goal(global_data, world_tran):
    """@brief - Get armtip goal pose in the arms coordinate system from hand world goal pose.
    This is useful to convert between a hand goal in the world coordinate system to a cartesian goal that
    can be sent directly to the staubli controller. 
    
    @param world_tf - tf of hand goal pose in world coordinates
    """
    try:    
        world_tf = pm.toTf(pm.fromMatrix(world_tran))
        bc = tf.TransformBroadcaster()
        now = rospy.Time.now()
        bc.sendTransform(world_tf[0], world_tf[1], now, "hand_goal", "world")
        #listener = tf.TransformListener()
        global_data.listener.waitForTransform("armbase", "arm_goal", now, rospy.Duration(1.0))
        armtip_robot = global_data.listener.lookupTransform('armbase', 'arm_goal', now)
        
        return pm.toMatrix(pm.fromTf(armtip_robot))
    except Exception, e:
        handle_fatal('convert_cartesian_world_goal failed: error %s.'%e) 
   
    


def send_cartesian_world_goal(global_data, world_tran, blocking = False):
    """@brief Command a cartesian move of the hand in the world's coordinate system.
    If blocking is set, returns 0 if movement was impossible, 1 if linear movement was possible, 2 if only nonlinear movement was possible
    Otherwhise, returns 1 if goal could be set, 0 otherwise. 
        
    Command the end effector of the robot to move to a certain position by whatever path it choses. Will move in straight lines if possible.
    This function assumes that a send_cartesian_goal function exists and that the robot's intrinsic coordinate system
    is given as 'armbase' in the TF graph. 

    WARNING: DOES NOT USE TRAJECTORY PLANNING OR COLLISION DETECTION

    @param world_tran - Desired hand position in world coordinates
    @blocking - decides whether to return immediately or wait for movement to finish
    

    """
    arm_goal = convert_cartesian_world_goal(global_data, world_tran)
    send_cartesian_goal(arm_goal, blocking)


def convert_cartesian_relative_goal(global_data, move_tran):
    """@brief - Convert a position relative to the hand's current coordinate system to the robot's base coordinate system.
    @param move_tran - the relative position in the hand's coordinate system. 

    """
    try:
        move_tf = pm.toTf(pm.fromMatrix(move_tran))
        print move_tf
        bc = tf.TransformBroadcaster()
        now = rospy.Time.now()
        bc.sendTransform(move_tf[0], move_tf[1], now, "hand_goal", "hand")
        #listener = tf.TransformListener()
        global_data.listener.waitForTransform("armbase", "arm_goal", now, rospy.Duration(1.0))
        armtip_robot = global_data.listener.lookupTransform('armbase','arm_goal', now)

        #arm_robot = global_data.listener.lookupTransform('armbase', 'arm', now)
        return pm.toMatrix(pm.fromTf(armtip_robot))
    except Exception, e:
        handle_fatal('convert_cartesian_relative_goal failed: error %s.'%e)
            

def send_cartesian_relative_goal(global_data, move_tran, blocking = False):
    """@brief - move the hand relative to the current hand position
     If blocking is set, returns 0 if movement was impossible, 1 if linear movement was possible, 2 if only nonlinear movement was possible
    Otherwhise, returns 1 if goal could be set, 0 otherwise.
    
    This function relies on convert_cartesian_relative_goal.
    
    WARNING: DOES NOT USE TRAJECTORY PLANNING OR COLLISION DETECTION 
    
    @param move_tran - the transform to move by in the hand coordinate system
    @param blocking - boolean flag for whether the command should wait for
    the robot to finish moving before returning. 
    """
    arm_goal = convert_cartesian_relative_goal(global_data, move_tran)
    return send_cartesian_goal(arm_goal, blocking)
    

def load_table( or_env ):
    """@brief Load a table whose base is the world origin
    @param or_env - OpenRave environment to add table to. 
    """
    table_url = '/home/armuser/ros/rosbuild_src/RearmGraspit/models/obstacles/zeroplane.iv'
    #graspit uses mm instead of meters
    kb = or_env.ReadTrimeshFile(table_url)
    kb.vertices /= 1000
    k = RaveCreateKinBody(or_env, "")
    k.SetName("table")
    k.InitFromTrimesh(kb, True)
    or_env.AddKinBody(k)

def regenerate_ikmodel():
    global_data = GlobalData()
    global_data.or_env = Environment()
    staubli = check_or_load_staubli( global_data.or_env )
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(staubli,iktype=IkParameterization.Type.Transform6D)
    ikmodel.autogenerate()


def SetupStaubliEnv(debug = False, robot_active = True, no_ros = False):
    """@brief - set up the staubli environment for planning
    @param debug - boolean flag to determine whether to visualize the openrave
    environment. 
    """
    if rospy.get_name() =='/unnamed' and not no_ros:
        rospy.init_node('trajectory_planner_node')
#    global global_data
    global_data = GlobalData()
    global_data.or_env = []
    #FIXME: Lame test for collision checker loading correctly
    while global_data.or_env == []:
        global_data.or_env = Environment()
        if not global_data.or_env.GetCollisionChecker():
            del global_data.or_env
            global_data.or_env = []        
    
    staubli = check_or_load_staubli( global_data.or_env )
    
    table = load_table( global_data.or_env )
    listener = []
    """Set the position of the arm relative to the world in openrave"""
    try:
        listener = tf.TransformListener()
        time.sleep(1)
        staubli_in_world_tf = listener.lookupTransform('/armbase','/world', rospy.Time(0))    
        staubli_in_world = pm.toMatrix(pm.fromTf(staubli_in_world_tf))
        staubli.SetTransform(linalg.inv(staubli_in_world))
    except:
        pass
    """Stop the base from worrying about collisions """
    b = staubli.GetLinks()
    b[0].Enable(False)
    """Load the manipulation problem to enable ik solutions
    """
    RaveLoadPlugin('/home/armuser/openrave/plugins/comps/plugins/libGeneralIK.so')
    RaveLoadPlugin('/home/armuser/openrave/plugins/comps/plugins/libcbirrt.so')
    RaveLoadPlugin('/home/armuser/openrave/plugins/comps/plugins/libmanipulation.so')

    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(staubli,iktype=IkParameterization.Type.Transform6D)
    ikmodel.load('/home/armuser/.openrave/kinematics.6daf9c2558dc15e66cbf2e7ceff5a950/ikfast41.Transform6D.x86_64.0_1_2_3_4_5.so')

    """If the robot you are using does not have an ikfast solution already generated, uncomment this line of code
    FIXME:This should be automatically detected.
    ikmodel.autogenerate()
    """
    """Open the viewer if desired """
    if debug:
        global_data.or_env.SetViewer('qtcoin')
        
    probs_manip = RaveCreateProblem(global_data.or_env,'Manipulation')
    staubli.GetManipulators()[0].SetIkSolver(ikmodel.iksolver)
    global_data.or_env.LoadProblem(probs_manip, staubli.GetName())
    global_data.probs_manip = probs_manip

    """Store the TF listener"""
    global_data.listener = listener

    """Update the current joint position of the robot in the world"""
    if robot_active:
        update_robot(staubli)
    return global_data 
    

def get_ee_tran(rob):
    """@brief - Get end effector transform of the robot in open rave.
    !!This is often not the hand transform, so be careful.

    @param rob - OpenRave robot to find the end effector transform for. 
    """
    return rob.GetManipulators()[0].GetEndEffectorTransform()

    
def test_cbirrt_with_tran(global_data):
    """@brief - Unit test to move robot back to it's current position.
    FIXME - test this function
    """
    tran = get_staubli_cartesian_as_tran()
    [s,f, dof_traj_list, j] = run_cbirrt_with_tran(global_data.or_env, tran)
    return dof_traj_list

                                                                                                                                               
def warn(str):
    print str
    rospy.logwarn(str)
    

def get_object_height(or_env, object_name = 'object', col = 2, filename = ''):    
    """@brief - Get the height of from the bottom of the in the
    to the origin of the object. Loads object if necessary, otherwise uses existing object. THIS FUNCTION IS A HACK
    This is to compensate for the current object localization scheme finding the location of the base of the object, not the location of the origin of the object.
    
    @param filename - name of file storing the object.
    @param global_data - a structure containing a member named or_env that
                         is a valid OpenRave environment.
    @param col - the column in the x,y,z list of points which contains the
                 'height' of this object. 2 implies the object is usually placed with
                 the z direction aligned with the up direction in the world.

    @param object_name - The name which the expected object has in the openrave environment. If it does not exist, the function will attempt to load the file filename and put the object here.
    

    """
    #warnings.warn('get_object_height is deprecated.', DeprecationWarning)
    k = []    
    k = or_env.GetKinBody(object_name)
    if not k:
        try:
            tm = or_env.ReadTrimeshFile(filename)
            if max(tm.vertices[:,col]) > 2.0:
                tm.vertices /= 1000.0
            k = RaveCreateKinBody(or_env,"")
            k.SetName(object_name)
            k.InitFromTrimesh(tm, True)
            or_env.AddKinBody(k)
        except Exception, e:
            warn('Failed to load object to get object height -\
            object %s, filename %s, error: %s'%(object_name, filename,e))
            return [],[]
    else:
        tm = k.GetLinks()[0].GetGeometries()[0].GetCollisionMesh()
    return -min(tm.vertices[:,col]), k


def add_object_to_planner(global_data, object_name = 'object', filename = '' ):
    """@brief - Add an object to the openrave planner.

    @param or_env - OpenRave environment object
    @param object_name - Name of the object, assumed to be the same as the object's frame in the TF tree
    
    
    """
    k = []    
    k = global_data.or_env.GetKinBody(object_name)
    if not k:
        try:
            tm = global_data.or_env.ReadTrimeshFile(filename)
            if amax(tm.vertices) > 2.0:
                tm.vertices /= 1000.0
            k = RaveCreateKinBody(global_data.or_env,"")
            k.SetName(object_name)
            k.InitFromTrimesh(tm, True)
            global_data.or_env.AddKinBody(k)
        except Exception, e:
            warn('Failed to load object - object %s, filename %s, error: %s'%(object_name, filename,e))
            return [],[]
    #Get the object transform in the world. 
    obj_tran = pm.toMatrix(pm.fromTf(global_data.listener.lookupTransform("/world", object_name, rospy.Time(0))))
    k.SetTransform(obj_tran)

    

def publish_true_object_tran(height):
    """@brief - A debugging function to visualize the origin of the objects
    in from openrave in the scene graph. 

    @param height - the offset from the table to the origin of the object.
    """
    try:
        object_height_offset = eye(4)
        object_height_offset[2,3] = height
        object_height_tf = pm.toTf(pm.fromMatrix(object_height_offset))
        bc = tf.TransformBroadcaster()
        bc.sendTransform(object_height_tf[0], object_height_tf[1], rospy.Time.now(), "/true_obj_pose", "/object")
    except Exception,e:
        rospy.logwarn("failed to publish true object height: %s"%e)
        return []
    return object_height_offset





def get_pregrasp_tran_from_tran(grasp_tran, dist):
    """@brief - convert from final grasp transform to pregrasp transform.

    This function currently just backs away along an approach direction. 

    @param grasp_tran - the final position of the hand in world coordinates.
    @param dist - the distance to back away from the object
    """
    e = eye(4)
    e[2,3] = dist
    return dot(grasp_tran, e)





def publish_grasp_tran(tran):
    """@brief - Add a grasp relative to the world origin to the scene graph
    @param tran - the grasp transform., 
    """
    bc = tf.TransformBroadcaster()
    ttf = pm.toTf(pm.fromMatrix(tran))
    bc.sendTransform(ttf[0], ttf[1], rospy.Time.now(),  "/hand_goal_pose", "/world")  
                 



def approach_to_contact(global_data, dist = .05):
    """@brief - Currently wrapper around move_forward_to_contact that drops the extraneous global_data and always blocks.

    WARNING: Currently only uses force torque transducer at wrist.
    
    TODO:Implement gaurd using tactile and strain gauges
    
    """
#    tm = get_tactile_threshold()
    return move_forward_to_contact(dist, True)
    



def release_exp():
    """@brief - Release a grasped objected.

    Raises hand, releases object, moves hand back 3 cm, moves hand down 3cm, moves hand back 7cm
    
    """    
    lift_arm(.03)
    open_barrett()
    move_forward(-.03)
    lift_arm(-.03)
    move_forward(-.07)
    



def pregrasp_object(global_data, object_name, adjust_height, grasp_tran, object_height_col = 2,
                    pregrasp_dist = -.05, object_height_adjustment = 0.0, run_it = True):
    """@brief - Move the hand to the pregrasp position. This procedure is suprisingly intricate.

    Given the object filename and the grasp transform, this transforms the grasp to world coordinates
    then runs a planner to avoid collisions in moving the arm the long distance to the
    object. This uses the scene graph, which is currently provided by a camera.

    Returns whether the pregrasp location was reachable, the filename in which the trajectory from OpenRave
    is stored, and the list of the dof values that are contained in the trajectory.
    """


    """ Sets up object location in openrave -- this code is all to compensate for the current object localization
    method only knowing the location of the base of the object - the height of the object's origin from the base
    must be used to adjust it's location in openrave before planning. 
    """

    print "Pregrasping object: %s"%(object_name)


    #Get the object transform in the world. 
    obj_tran = pm.toMatrix(pm.fromTf(global_data.listener.lookupTransform("/world",object_name, rospy.Time(0))))

    if (obj_tran == []):
        warn("Scene graph in TF is broken! Most\
           likely, the object transform could not be found.")
        return [],[], [], []
                

    """Set up and visualize the grasp transforms"""

    pregrasp_tran = get_pregrasp_tran_from_tran(grasp_tran, pregrasp_dist)
    publish_grasp_tran(pregrasp_tran)
    final_tran = dot(obj_tran, pregrasp_tran)
    publish_grasp_tran(final_tran)

    success = False
    """Run the planner"""
    plan = []
    j = []
    try:
        success, trajectory_filename, dof_list, j = run_cbirrt_with_tran( global_data.or_env, final_tran )

    except Exception,e:
        warn("Failed running planner with\
                       error %s"%e)
    if not j:
        warn("no solution\n")
        



    """Run the robot"""
    if success and run_it:
        success, client = run_staubli_on_trajectory_point_message( dof_list , True )
        if not success:
            rospy.logwarn("Running staubli on trajectory message failed with results %i"%(client.get_state()))

        """ Make sure robot moves to it's final joint location. This is to compensate for the blended trajectory
        on the staubli not always finishing it's final motion to the end point of the joint trajectory.
        FIXME: Deal with this on the staubli end of things

        """
        
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ABORTED:
            success,client,j = run_staubli_on_joint_goal(dof_list[-1],True)        



    return success, final_tran, dof_list, j


def release_gently(global_data, max_dist, ignore_warning = 0):
    """ @brief - Gently place the object on the table by to at most the maximum distance
        and dropping the object
        
        @param global_data - the data structure storing the openrave environment and other global
                             parameters
        @param max_dist - The highest above the table we allow the arm to be before releasing

        TODO: Test and debug this function!!!!
    """

    """ This function is not ready yet!"""
    rospy.log_warn("release gently function called, but it is not done being developed yet! ")
    if not ignore_warning:
        return

    success = True
    reason = "succeeded releasing"
    #find hand position with respect to world
    try:
        hand_in_world = pm.toMatrix(pm.fromTf(
            listener.lookupTransform('/hand','/world', rospy.Time(0))))
    except:
        success = False
        reason = "failed to find hand in world"
    #Find distance above table and move hand down by that distance
    if success:
        try:
            move_dist = hand_in_world[2,3] - max_dist
            if move_dist > 0:            
                lift_arm_to_contact(-move_dist)
        except:
            success = False
            reason = "failed to move arm down"
    #open the hand to drop the object
    if success:
        try:
            success, reason, position = open_barrett()
        except:
            success = False
            reason = "open barrett command threw exception"
            
    return success, reason
        
        
    
    
def test_calibration(global_data, height = .1, run_it = False):
    """@brief - test the calibration of the hand by putting it over the world facing down
       @param global_data - a structure containing an openrave environment in the or_env attribute
       @param height - how high the palm of the hand should be above the world origin in meters
       @run_it - whether the motion should just be planned or also run.
    
    """

    #create a transform putting the hand over the origin but upside down.
    desired_hand_tran = eye(4)
    desired_hand_tran[1,1] = -1
    desired_hand_tran[2,2] =  -1
    desired_hand_tran[2,3] = height
    try:
        success, trajectory_filename, dof_list, j = run_cbirrt_with_tran( global_data.or_env, desired_hand_tran)
    except Exception,e:
        warn("Failed running planner with\
                       error %s"%e)
        pdb.post_mortem()
    
    if success and run_it:
        success, client = run_staubli_on_trajectory_point_message( dof_list , True )
    return success, desired_hand_tran, dof_list, j


def send_model_rec():
    """@brief - Helper function to request that the vision system reinitialize. 

    """
    from std_msgs.msg import String, Empty
    p = rospy.Publisher("/graspit/refresh_models", Empty)
    p.publish()
    
    
