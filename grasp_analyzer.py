from sklearn import *
import rospy
import random
import trajectory_planner as tp
import graspit_msgs.msg
import itertools
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, unique
from math import acos
import pdb
import std_msgs.msg

class GraspAnalyzer(object):
    def __init__(self, global_data, analyze_grasp_topic = "/graspit/analyze_grasps", demo_pose_topic = "/graspit/analyze_demo_pose"):
        self.data = set()
        self.grasp_analysis_func = []
        self.retrain_threshold = 1
        self.model = neighbors.NearestNeighbors(n_neighbors=20, algorithm = 'kd_tree') 
        self.global_data = global_data
        self.max_data_len = 5000
        self.array_data = []
        self.grasp_classes = []
        self.analyze_grasp_subscriber = rospy.Subscriber(analyze_grasp_topic, graspit_msgs.msg.Grasp, self.analyze_grasp, queue_size = 1)
        self.analyze_pose_subscriber = rospy.Subscriber(demo_pose_topic, graspit_msgs.msg.Grasp, self.analyze_demonstration_pose, queue_size = 1)
        self.grasp_analysis_publisher = rospy.Publisher(analyze_grasp_topic + "_results", graspit_msgs.msg.GraspStatus)
        self.demo_pose_analysis_publisher = rospy.Publisher(demo_pose_topic + "_results", std_msgs.msg.Float32)


    
    def data_to_array(self):
        self.array_data = array([pm.toMatrix(pm.fromMsg(grasp_msg[0].final_grasp_pose))[:3,3]
                                       for grasp_msg in self.data])
        

        
    
    def analyze_demonstration_pose(self, demo_grasp):
        if(len(self.data) ==0):
            return 1.0
        
        demo_pose = pm.toMatrix(pm.fromMsg(demo_grasp.final_grasp_pose))
        demo_position = demo_pose[:3,3]
        
        distances, indices = self.model.kneighbors(demo_position)
        indices = unique(indices)
        nbrs = [t for t in itertools.compress(self.data, indices)]
        valid_nbrs = []
        for n in nbrs:
            pose = pm.toMatrix(pm.fromMsg(n[0].final_grasp_pose))
            if (acos(dot(pose[:3,2],demo_pose[:3,2]) < .52)):
                valid_nbrs.append(n)
        
        if len(valid_nbrs):
            success_probability = len([n for n in valid_nbrs if n[1] & 1])/(1.0*len(valid_nbrs))            
        else:
            success_probability = 0
        self.demo_pose_analysis_publisher.publish(success_probability)
        return success_probability


        
        
    def train_model(self, grasp, grasp_class):
        
        self.data.add((grasp, grasp_class))
        if len(self.data) > self.max_data_len:
            self.sparcify_data()
            
        if not len(self.data)%self.retrain_threshold:
            self.data_to_array()
            self.model.fit(self.array_data)

    def sparcify_data(self):
        self.data = random.sample(self.data, len(self.data)/2.0)

    

    def test_grasp_msg(self, grasp_msg):
        """@brief
        """
        def set_home(rob):
            rob.SetActiveDOFs(range(10))
            rob.SetActiveDOFValues([0,0,0,0,0,0,0,0,0,0])
            
        def set_barrett_hand_open(rob):
            rob.SetActiveDOFs(range(6,10))
            rob.SetActiveDOFValues([0,0,0,0])

        def set_barrett_hand_dofs(rob, dofs):
            rob.SetActiveDOFs(range(6,10))
            rob.SetActiveDOFValues(dofs)
            
        def set_staubli_dofs(rob, dofs):
            rob.SetActiveDOFs(range(6))
            rob.SetActiveDOFValues(dofs)


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
            if j != []:
                return graspit_msgs.msg.GraspStatus.SUCCESS
            
            #Test if pose is reachable if we ignore collisions all together
            j = rob.GetManipulators()[0].FindIKSolutions(tran, 0)
            if j != []:
                return graspit_msgs.msg.GraspStatus.UNREACHABLE
            else:
                return (graspit_msgs.msg.GraspStatus.UNREACHABLE | graspit_msgs.msg.GraspStatus.ENDEFFECTORERROR)
                

        def test_trajectory_reachability(tran):
            success, trajectory_filename, dof_list, j = tp.run_cbirrt_with_tran( self.global_data.or_env, tran, [], 1 )
            return success


        def set_barrett_enabled(rob, collision):
            links = rob.GetManipulators()[0].GetChildLinks()
            [l.Enable(collision) for l in links[1:]]
             

        def test_pregrasp_to_grasp(rob, pre_grasp, grasp, steps):
            grasp_diff = (grasp - pre_grasp)/steps
            test_reachability_result = graspit_msgs.msg.GraspStatus.SUCCESS
            for i in xrange(1, steps):
                test_tran = pre_grasp + i * grasp_diff
                test_reachability_result = test_pose_reachability(rob, test_tran)
                if test_reachability_result is not graspit_msgs.msg.GraspStatus.SUCCESS:
                    print "Failed in test pregrasp to grasp in step %i in pose %s with result %i\n"%(i, grasp_tran, test_reachability_result)
                    break
            return test_reachability_result

        def test_hand_closing(rob, start_dof, end_dof, steps):
            dof_diff = (array(end_dof) - array(start_dof))/steps
            #self.global_data.or_env.GetCollisionChecker().SetCollisionOptions(tp.CollisionOptions.ActiveDOFs)                
            for i in xrange(1, steps):
                dofs = start_dof + i * dof_diff
                set_barrett_hand_dofs(rob, dofs)
                rob.GetEnv().UpdatePublishedBodies()
                end_effector_collision =  rob.GetEnv().CheckCollision(rob)
                #raw_input('continue to next test')
                if end_effector_collision:
                    return False
            return True

        def test_grasp_closure(rob, grasp_msg):
            pre_grasp_dof = [0,0,0,grasp_msg.pre_grasp_dof[0]]
            grasp_dof = [grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]]
            return test_hand_closing(rob, pre_grasp_dof, grasp_dof, 10)
            

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
            target_object.Enable(False)
            grasp_test = test_pose_reachability(robot, grasp_tran)
            if grasp_test is not graspit_msgs.msg.GraspStatus.SUCCESS:
                return graspit_msgs.msg.GraspStatus.FAILED, graspit_msgs.msg.GraspStatus.GRASPERROR, grasp_test

            move_forward_test = test_pregrasp_to_grasp(robot, pre_grasp_tran, grasp_tran, 10)
            if move_forward_test is not graspit_msgs.msg.GraspStatus.SUCCESS:
                return graspit_msgs.msg.GraspStatus.FAILED, (graspit_msgs.msg.GraspStatus.PREGRASPERROR | graspit_msgs.msg.GraspStatus.GRASPERROR), move_forward_test
            
            j = robot.GetManipulators()[0].FindIKSolutions(grasp_tran, tp.IkFilterOptions.CheckEnvCollisions)
            if not j:
                pdb.set_trace()
            set_staubli_dofs(robot, j[0])
            closure_result  = test_grasp_closure(robot, grasp_msg)

            if not closure_result:
                return graspit_msgs.msg.GraspStatus.FAILED, (graspit_msgs.msg.GraspStatus.PREGRASPERROR | graspit_msgs.msg.GraspStatus.GRASPERROR), graspit_msgs.msg.GraspStatus.ENDEFFECTORERROR
            
            target_object.Enable(True)

            set_home(robot)
            #Can we reach the pregrasp pose
            trajectory_test = test_trajectory_reachability(pre_grasp_tran)
            
             
            if not trajectory_test:
                return graspit_msgs.msg.GraspStatus.FAILED, graspit_msgs.msg.GraspStatus.ROBOTERROR, graspit_msgs.msg.GraspStatus.PREGRASPERROR

        return graspit_msgs.msg.GraspStatus.SUCCESS, 0, 0


    def analyze_grasp(self, grasp_msg):
        success, failure_mode, score =  self.test_grasp_msg(grasp_msg)
        gs = graspit_msgs.msg.GraspStatus()
        if not success:
            gs.status_msg =  "Grasp %i unreachable: %i %i %i"%(grasp_msg.secondary_qualities[0], success, failure_mode, score)
            print gs.status_msg
        gs.grasp_identifier = grasp_msg.secondary_qualities[0]
        gs.grasp_status = success | failure_mode | score

        self.train_model(grasp_msg, gs.grasp_status)
        self.grasp_analysis_publisher.publish(gs)        
