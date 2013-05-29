#!/usr/bin/env python
import roslib; roslib.load_manifest('trajectory_planner')
import rospy
import threading

from trajectory_planner import *
from numpy import *
from HaoUtil import *

from graspit_ros_server.srv import QueryExperience
import pr_msgs


######## Tactile ########
tactileMsg = None
tactileMonitor = None

def TactileMonitorCallback(tactile_msg):
    global tactileMsg
    tactileMsg = tactile_msg

def StartTactileMonitoring():
    global tactileMonitor
    tactileMonitor = rospy.Subscriber("/bhd/tactile", pr_msgs.msg.BHTactile, TactileMonitorCallback)

def StopTactileMonitoring():
    global tactileMsg
    global tactileMonitor
    tactileMonitor.unregister()
    tactileMsg = None
    tactileMonitor = None

######## Hand state ########
handStateMsg = None
handStateMonitor = None

def HandStateCallback(handstate_msg):
    global handStateMsg
    handStateMsg = handstate_msg

def StartHandStateMonitoring():
    global handStateMonitor
    handStateMonitor = rospy.Subscriber("bhd/handstate", pr_msgs.msg.BHState, HandStateCallback)

def StopHandStateMonitoring():
    global handStateMsg
    global handStateMonitor
    handStateMonitor.unregister()
    handStateMsg = None
    handStateMonitor = None

def QueryGraspExperience():
    global tactileMsg
    global handStateMsg
    pdb.set_trace()
    try:
        query_experience = rospy.ServiceProxy('grasp_experience', QueryExperience)
        res = query_experience(tactileMsg, handStateMsg)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

######## Robot Control ########
def AdjustHand(adjustment):
    '''
    Move the hand in the current palm coordinate system by the transform specified in adjustment
    '''
    cancel_arm_motion()
    current_arm_tran = get_staubli_cartesian_as_tran()

    q = array([adjustment.orientation.x,
               adjustment.orientation.y,
               adjustment.orientation.z,
               adjustment.orientation.w])
    p = array([adjustment.position.x / 1000.0,
               adjustment.position.y / 1000.0,
               adjustment.position.z / 1000.0])
    offset_tran = GetTransfFromQuaternionPosition(q, p)
    ee_tran = dot(current_arm_tran, offset_tran)
    SendCartesianGoal(ee_tran, False)
    #wait for as long as 60 seconds
    WaitForArm(60)


if __name__ == '__main__':
    NODE_NAME='blind grasping'

    global_data, ftm, tm = start()
    rospy.loginfo('initialized')
    pdb.set_trace()
    rospy.wait_for_service('grasp_experience')
    pdb.set_trace()

    StartTactileMonitoring()
    StartHandStateMonitoring()

    #guarded close fingers

    #query experience database and find an adjustment
    res = QueryGraspExperience()
    MoveHandSrv(1,[0,0,0,0])
    AdjustHand(res.adjustment)
    MoveHandSrv(1,[2,2,2,0])
    


    
    StopTactileMonitoring()
    StopHandStateMonitoring()
