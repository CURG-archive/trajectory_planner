from grasp_message_robot_server import *

def test_graspit_send_multiobjects():
    rospy.init_node('grasp_message_server_test')
    ge = GraspExecutor(False)
    ge.model_manager.refresh()
    ge.publish_table_models()
    return ge

def set_staubli_set_pose(ge):
    staubli = ge.global_data.or_env.GetRobots()[0]
    staubli_test_tran = tp.eye(4)
    staubli_test_tran[2,3] = .2
    staubli.SetTransform(staubli_test_tran)


def add_barrett(ge):
    barrett = ge.global_data.or_env.ReadRobotXMLFile("/home/armuser/openrave/openrave/robots/barretthand.robot.xml")
    ge.global_data.or_env.AddRobot(barrett)
    disable_entire_barrett(barrett)
    return barrett

def disable_entire_barrett(barrett):
    links = barrett.GetLinks()
    for link in links:
        link.Enable(False)

def demonstrate_grasp_position(ge, barrett, grasp_tran, object_name, pregrasp_dist = -0.05):
    global_data = ge.global_data
    ge.publish_table_models()
    obj_tran = pm.toMatrix(pm.fromTf(global_data.listener.lookupTransform("/world",object_name, rospy.Time(0))))
    pregrasp_tran = tp.get_pregrasp_tran_from_tran(grasp_tran, pregrasp_dist)
    tp.publish_grasp_tran(pregrasp_tran)
    final_tran = dot(obj_tran, pregrasp_tran)
    barrett.SetTransform(final_tran)


def test_grasp_position_1(ge, barrett, object_ind = 0):
    grasp_tran = tp.array([[ 1. ,  0. ,  0. ,  0. ],[ 0. , -1. ,  0. ,  0. ],[ 0. ,  0. , -1. ,  0.4], [ 0. ,  0. ,  0. ,  1. ]])
    demonstrate_grasp_position(ge, barrett, grasp_tran, ge.model_manager.model_list[object_ind].object_name)
    
    
def test_pregrasp(ge, object_ind):
    grasp_tran = tp.array([[ 1. ,  0. ,  0. ,  0. ],[ 0. , -1. ,  0. ,  0. ],[ 0. ,  0. , -1. ,  0.4], [ 0. ,  0. ,  0. ,  1. ]])
    ge.publish_table_models()
    tp.pregrasp_object(ge.global_data, ge.model_manager.model_list[object_ind].object_name, adjust_height = False, grasp_tran = grasp_tran, run_it = False)
    
