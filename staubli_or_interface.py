import util
from barrett_manager import *
from staubli_manager import *

def check_or_load_staubli( or_env ):
    """@brief - make sure staubli is loaded into openrave environment
    @param or_env - openrave python environment
    """
    robot_list = or_env.GetRobots()
    current_robot = []

    if (len(robot_list) > 0):        
        
        print "trajectory_planner::run_cbirrt:: WARNING: too many robots in openrave environment \n"
        current_robot = robot_list[0]
    else:
        current_robot = or_env.ReadRobotXMLFile("/home/armuser/openrave/openrave/robots/staubli.robot.xml")
        print "robot loaded"
        if current_robot:
            or_env.AddRobot(current_robot)


    return current_robot


def dof_list_from_traj_file(filename, active_dof_list):
    """@brief - Open trajectory file and read dof trajectory

    @param filename - file containing trajectory from openrave
    @param active_dof_list - list of dofs used by this robot

    returns a list of dof lists for each point in the trajectory
    """
    f = open( filename )
    dof_traj_list = []
    f.readline()
    f.readline()

    #The first line of the file is metadata    

    for line in f:
        traj_list = util.str2num.str2num( line )
        dof_traj_point = []
        for n in active_dof_list:
            dof_traj_point.append( traj_list[n+1] )
        dof_traj_list.append( dof_traj_point )
    return dof_traj_list


def update_robot(rob):
    """@brief - Update the robot in OpenRave representation of the robot from the staubli manager

       @param rob - OpenRave robot object.
    """
    rob.SetActiveDOFs(range(6))
    j = get_staubli_joints()
    rob.SetActiveDOFValues(j)
    try:
        barret_joints = get_barrett_joints()
        rob.SetActiveDOFs(range(6,10))
        rob.SetActiveDOFValues(barret_joints)
    except:
        pass

    rob.SetActiveDOFs(range(6))
    return True
