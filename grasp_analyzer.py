#from numpy.scikit import *
import rospy
import random
import trajectory_planner as tp
import graspit_msgs.msg
import itertools
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs


class GraspAnalyzer(object):
    def __init__(self, global_data, analyze_grasp_topic = "/graspit/analyze_grasps"):
        self.data = []
        self.grasp_analysis_func = []
        self.retrain_threshold = 5
        #self.model = neighbor.NearestNeighbor(n_neighbors=3, algorithm = 'kd_tree') 
        self.global_data = global_data
        self.max_data_len = 5000
        self.array_data = []
        self.grasp_classes = []
        self.analyze_grasp_subscriber = rospy.Subscriber(analyze_grasp_topic, graspit_msgs.msg.Grasp, self.analyze_grasp, queue_size = 20)
        self.grasp_analysis_publisher = rospy.Publisher(analyze_grasp_topic + "_results", graspit_msgs.msg.GraspStatus)

    """
    def data_to_array(self):
        self.array_data = numpy.array([pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))[:3,2]
                                       for grasp_msg in self.data])

        
    
    def analyze_demonstration_pose(self, demonstration_pose):
        distances, indices = self.model.kneighbors([demonstration_pose])
        nbrs = itertools.compress(self.data, indices)
        success_probability = len[n for n in nbrs if n & 1]/(1.0*len(nbrs))
        return success_probability

    def analyze_grasp(self, grasp):
        grasp_class = self.test_grasp_msg(grasp)
        self.train_model(grasp, grasp_class)
        
        
    def train_model(self, grasp, grasp_class):
        self.data.append([grasp, grasp_class])
        if len(self.data) > self.max_data_len:
            self.sparcify_data()
            
        if not len(data)%self.retrain_threshold:
            self.data_to_array()
            self.model.fit(self.array_data)

    def sparcify_data(self):
        self.data = random.sample(self.data, len(self.data)/2.0)

    """

    def test_grasp_msg(self, grasp_msg):
        """@brief
        """
        def set_home(rob):
            rob.SetActiveDOFs(range(10))
            rob.SetActiveDOFValues([0,0,0,0,0,0,0,0,0,0])
            
        def set_barrett_hand_open(rob):
            rob.SetActiveDOFs(range(6,10))
            rob.SetActiveDOFValues([0,0,0,0])                

        def grasp_tran_from_msg(grasp_msg):
            grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
            grasp_tran[0:3,3] /=1000 #mm to meters
            return grasp_tran

        def test_pose_reachability(rob, tran):
            #Test if end effector pose is in collision
            rob.SetActiveDOFs(range(6))
            end_effector_collision =  rob.GetManipulators()[0].CheckEndEffectorCollision(tran)
            if end_effector_collision:
                return graspit_msgs.msg.GraspStatus.ENDEFFECTORERROR
            
            #Test if pose is reachable
            j = rob.GetManipulators()[0].FindIKSolutions(tran, tp.IkFilterOptions.CheckEnvCollisions)
            if j is not []:
                return graspit_msgs.msg.GraspStatus.SUCCESS
            
            #Test if pose is reachable if we ignore collisions all together
            j = rob.GetManipulators()[0].FindIKSolutions(tran, 0)
            if j is not []:
                return graspit_msgs.msg.GraspStatus.UNREACHABLE

        def test_trajectory_reachability(tran):
            success, trajectory_filename, dof_list, j = tp.run_cbirrt_with_tran( self.global_data.or_env, tran, [], 1 )
            return success


        def set_barrett_enabled(rob, collision):
            links = rob.GetManipulators()[0].GetChildLinks()
            [l.Enable(collision) for l in links[1:]]
             
            

        with self.global_data.or_env:
            robot = self.global_data.or_env.GetRobots()[0]
            set_home(robot)            

            target_object = self.global_data.or_env.GetKinBody(grasp_msg.object_name)

            obj_tran = target_object.GetTransform()

            grasp_rel_tran = grasp_tran_from_msg(grasp_msg)
            grasp_tran = dot(obj_tran, grasp_rel_tran)
            pre_grasp_tran = dot(obj_tran, tp.get_pregrasp_tran_from_tran(grasp_rel_tran, -.05))
            
            pregrasp_test = test_pose_reachability(robot, pre_grasp_tran)
            if pregrasp_test is not graspit_msgs.msg.GraspStatus.SUCCESS:
                return graspit_msgs.msg.GraspStatus.FAILED, graspit_msgs.msg.GraspStatus.PREGRASPERROR, pregrasp_test

            
            #Can we reach the grasp pose
            #Disable target object collisions
            #Can we reach the pregrasp pose
            target_object.Enable(False)
            grasp_test = test_pose_reachability(robot, grasp_tran)
            if grasp_test is not graspit_msgs.msg.GraspStatus.SUCCESS:
                return graspit_msgs.msg.GraspStatus.FAILED, graspit_msgs.msg.GraspStatus.GRASPERROR, grasp_test
            target_object.Enable(True)
            
            trajectory_test = test_trajectory_reachability(pre_grasp_tran)
            
             
            if not trajectory_test:
                return graspit_msgs.msg.GraspStatus.FAILED, graspit_msgs.msg.GraspStatus.ROBOTERROR, graspit_msgs.msg.GraspStatus.PREGRASPERROR

        return graspit_msgs.msg.GraspStatus.SUCCESS, 0, 0

    def analyze_grasp(self, grasp_msg):
        success, failure_mode, score =  self.test_grasp_msg(grasp_msg)
        gs = graspit_msgs.msg.GraspStatus()
        if not success:
            gs.status_msg =  "Grasp unreachable: %i %i %i"%(success, failure_mode, score)
            print gs.status_msg
        gs.grasp_identifier = grasp_msg.secondary_qualities[0]
        gs.grasp_status = success | failure_mode | score
        self.grasp_analysis_publisher.publish(gs)
