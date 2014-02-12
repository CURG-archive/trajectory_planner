from trajectory_planner import *
from tf_conversions import posemath as PoseMath
from numpy import ones,zeros, linspace
from actionlib_msgs.msg import *
from std_msgs.msg import *
from time import sleep
from staubli_tx60.srv import ResetMotion
from math import pi

global_data = dict()

def SetHome(global_data):
    pose_msg = PoseMath.fromMsg(get_staubli_cartesian_as_pose_msg())
    global_data['Home'] = PoseMath.toMatrix(pose_msg)

def one_dimensional_dither(global_data, bound, step_num, dim, forward_move_dist):
    home_mat = global_data['Home']
    TareForceTorque()
    base_euler = zeros([6,1])
    forward_movement = eye(4)
    forward_movement[2,3] = forward_move_dist
    #assume movement is given in hand coordinates.  Apply T to handtran to get it in arm coordinates
    hand_tran = tf.transformations.euler_matrix(0, 0, 20.0/180.0 * pi, 'rxyz')
    for step in linspace(-bound, bound, step_num, endpoint = True):
        euler = base_euler.copy()
        euler[dim] += step
        euler[0:3] = dot(hand_tran[:3,:3], euler[0:3])
        T = tf.transformations.euler_matrix( euler[3],euler[4], euler[5], 'rxyz' )
        T[0:3,3] = euler[0:3,0]
        SendCartesianGoal(dot(home_mat,T), blocking = True)
        SetFTCallback(global_data,FTThresholdCallback)
        client = SendCartesianGoal(dot(home_mat, dot(forward_movement,T)), blocking = True)
        if client.get_state() == GoalStatus.SUCCEEDED and global_data.has_key('ft_sub'):
            break
        if global_data.has_key('ft_sub'):
            sub = global_data.pop('ft_sub')
            sub.unregister()
            del sub

        
def SetFTCallback(global_data, callback_func):
    if global_data.has_key('ft_sub'):
        sub = global_data.pop('ft_sub')
        sub.unregister()
        del sub
    global_data['ft_sub'] = rospy.Subscriber("ForceTorque/Readings", geometry_msgs.msg.Wrench, callback_func)
    


def TareForceTorque():
    if not global_data.has_key('ft_tare_pub'):
        global_data['ft_tare_pub'] = rospy.Publisher("ForceTorque/Tare", Empty)
    global_data['ft_tare_pub'].publish(Empty())
    sleep(1)
    return

delay_num = 5
hits = delay_num
threshold = -14

def FTThresholdCallback(wrench_msg):
    global hits
    global delay_num
    global threshold
    global global_data
    if wrench_msg.force.z < threshold:
        hits -= 1
    else:
        hits = delay_num
    if hits == 0:
        cancel_arm_motion()
        if global_data.has_key('ft_sub'):
            sub = global_data.pop('ft_sub')
            sub.unregister()
            del sub
    return

def cancel_arm_motion():
    reset_motion = rospy.ServiceProxy('/cancelMotion', ResetMotion)
    try:
        reset_motion()
    except Exception as e:
        rospy.logwarn("cancelMotion failed %s"%e)
    rospy.logwarn("motion cancelled")
    
                                                                
