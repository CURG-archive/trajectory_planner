#!/usr/bin/env python
import roslib
roslib.load_manifest( "trajectory_planner" )
import rospy
import time
import graspit_msgs.msg
import geometry_msgs.msg
import tf, tf_conversions, tf.transformations
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs
import tf_conversions.posemath as pm
from time import sleep 
import trajectory_planner as tp
import pdb
from object_filename_dict import file_name_dict
from std_msgs.msg import String, Empty
#from IPython.Shell import IPShellEmbed
from adjust_message_robot_server import AdjustExecutor, ShakeExecutor, MoveExecutor
from time import time
from model_rec_manager import *
import HaoUtil as hu

Hao = False

class GraspExecutor():
    """@brief - Generates a persistent converter between grasps published by a graspit commander and
    the trajectory planning module to actually lift objects off the table

    @member grasp_listener - subscriber to the graspit grasp channel. Expects graspit_msgs Grasp types
    @member name_listener - subscribes to an object name channel. Expects a string which corresponds
                            to an entry in the filename dictionary 
    @member global_data - Planning environment
    @member target_object_name - the current grasp target
    """

    def __init__(self, init_planner = True):
        self.grasp_listener = rospy.Subscriber("/graspit/grasps", graspit_msgs.msg.Grasp, self.process_grasp_msg)

        self.grasp_analyzer = rospy.Subscriber("/graspit/analyze_grasps", graspit_msgs.msg.Grasp, self.analyze_grasp)
        
        self.name_listener = rospy.Subscriber("/graspit/target_name", String, self.process_object_name)
        self.refresh_models_listener = rospy.Subscriber("/graspit/refresh_models", Empty, self.refresh_model_list)
        
        
        self.global_data = tp.SetupStaubliEnv(True, init_planner)
        
        
        
        self.model_manager = ModelRecManager(self.global_data.listener)
        self.graspit_status_publisher = rospy.Publisher("/graspit/status", graspit_msgs.msg.GraspStatus)
        self.graspit_target_publisher = rospy.Publisher("/graspit/target_name", String)
        self.graspit_object_publisher = rospy.Publisher("/graspit/object_name", graspit_msgs.msg.ObjectInfo)
        self.remove_object_publisher = rospy.Publisher("/graspit/remove_objects", String)
        self.target_object_name = "flask"

        self.last_grasp_time = 0
        self.table_cube=[geometry_msgs.msg.Point(-0.7,0,-0.02), geometry_msgs.msg.Point(0.2,1,1)]

    def process_object_name(self, string):
        self.target_object_name = string.data
    
    def refresh_model_list(self, empty_msg):

        
        self.model_manager.refresh()
        self.model_manager()
        #self.publish_target()
        self.remove_object_publisher.publish('ALL')
        self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()

    def publish_target(self):
        self.graspit_target_publisher.publish(self.model_manager.get_model_names()[0])

    def publish_table_models(self):
        self.model_manager()
        table_models = [model for model in self.model_manager.model_list if self.point_within_table_cube(model.get_world_pose().position)]

        print '\n'.join(['Model rejected %s'%(model.object_name) for model in self.model_manager.model_list if model not in table_models])
        
        for model in table_models:
            model()
            
            object_name = "%s %s"%(model.model_name, model.object_name)
            print "Sending object: %s \n"%(object_name)
            p = model.get_world_pose()
            print "Model name: %s"%(model.object_name)
            print p
            self.graspit_object_publisher.publish(object_name, p)
        self.graspit_status_publisher.publish(0, '')

    def clear_objects(self):
        model_name_list = []        
        model_name_list = [model.object_name for model in self.model_manager.model_list]
        self.remove_objects_publisher.publish(' '.join(model_name_list))

    def add_object_to_planner(self, model):
        tp.add_object_to_planner(self.global_data, model.object_name, file_name_dict[model.model_name])

    def add_all_objects_to_planner(self):
        for model in self.model_manager.model_list:
            self.add_object_to_planner(model)

    def remove_all_objects_from_planner(self):
        body_list = self.global_data.or_env.GetBodies()
        ignored_body_list = ['StaubliRobot', 'table']
        for body in body_list:
            if body.GetName() not in ignored_body_list:
                self.global_data.or_env.Remove(body)
                del body

    def test_grasp_msg(self, grasp_msg):

        def set_home(rob):
            rob.SetActiveDOFs(range(10))
            rob.SetActiveDOFValues([0,0,0,0,0,0,0,0,0,0])
            
        def set_barrett_hand_open(rob):
            rob.SetActiveDOFs(range(6,10))
            rob.SetActiveDOFValues([0,0,0,0])                

        def pregrasp_tran_from_msg(grasp_msg):
            grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
            grasp_tran[0:3,3] /=1000 #mm to meters
            return grasp_tran

        def test_pose_reachability(rob, tran):
            #Test if end effector pose is in collision
            rob.SetActiveDOFs(range(6))
            end_effector_collision =  rob.GetManipulators()[0].CheckEndEffectorCollision(tran)
            if end_effector_collision:
                return 1
            
            #Test if pose is reachable
            j = rob.GetManipulators()[0].FindIKSolutions(tran, IkFilterOptions.CheckEnvCollisions)
            if j is not []:
                return 0
            
            #Test if pose is reachable if we ignore collisions all together
            j = rob.GetManipulators()[0].FindIKSolutions(tran, 0)
            if j is not []:
                return 2

        def test_trajectory_reachability(tran):
            success, trajectory_filename, dof_list, j = tp.run_cbirrt_with_tran( self.global_data.or_env, tran, [], 1 )
            return success


        def set_barrett_enabled(rob, collision):
            links = rob.GetManipulators()[0].GetChildLinks()
            [l.Enable(collision) for l in links[1:]]
             
            
        robot = self.global_data.or_env.GetRobots()[0]
        with robot:
            set_home(robot)            

            target_object = global_data.or_env.GetKinBody(self.target_name)
            obj_tran = target_object.GetTransform()

            grasp_tran = dot(pregrasp_tran_from_msg(grasp_msg), obj_tran)
            pre_grasp_tran = dot(tp.get_pregrasp_tran_from_tran(grasp_tran, -.05),
                                 obj_tran)
            
            pregrasp_test = test_pose_reachability(robot, pre_grasp_tran)
            if pregrasp_test:
                return 0, 0, pregrasp_test

            
            #Can we reach the grasp pose
            #Disable target object collisions
            #Can we reach the pregrasp pose
            target_object.Enable(False)            
            grasp_test = test_pose_reachability(robot, grasp_tran)
            if grasp_test:
                return 0, 1, grasp_test
            target_object.Enable(True)
            
            trajectory_test = test_trajectory_reachability(pre_grasp_gran)
            
             
            if not trajectory_test:
                return 0, 2, 0

        return 1, 0, 0

    def analyze_grasp(self, grasp_msg):
        success, failure_mode, score =  self.test_grasp_msg(grasp_msg)
        if not success:
            print "Grasp unreachable: %i %i %i"%(success, failure_mode, score)
            

    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to grasp the object and lift it

        First moves the arm to pregrasp the object, then attempts to grasp and lift it.
        The grasp and lifting phase is currently completely open loop
        
        """
        with self.global_data.or_env:
            if (time() - self.last_grasp_time) < 30:
                return [], []
            self.last_grasp_time = time()
            print grasp_msg
            grasp_status = graspit_msgs.msg.GraspStatus.SUCCESS
            grasp_status_msg = "grasp_succeeded"
            success = 1
            if not tp.is_home():
                print 'go home'
                tp.go_home(self.global_data)
            #    if not success:
            #        grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
            #        grasp_status_msg = "Unable to go home"

            if success:
                success, grasp_status_msg, positions = tp.open_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            if success:
                grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
                grasp_tran[0:3,3] /=1000 #mm to meters
                tp.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
                tp.update_robot(self.global_data.or_env.GetRobots()[0])

                print 'pre-grasp'
                self.model_manager()
#            success, final_tran, dof_list, j = tp.pregrasp_object(self.global_data, file_name_dict[self.target_object_name],  grasp_tran)
                print 'after model_manager()'
                success, final_tran, dof_list, j = tp.pregrasp_object(self.global_data, self.target_object_name,  False, grasp_tran)
                print 'after pre-grasp'
                #raw_input("Press enter...")
                tp.update_robot(self.global_data.or_env.GetRobots()[0])
                if not success:
                    if not j:
                        grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                        grasp_status_msg = "Pregrasp tran Out of range!"
                    else:
                        grasp_status = graspit_msgs.msg.GraspStatus.FAILED
                        grasp_status_msg = "Unknown planner failure!"

        if success:
            if not Hao:
                success = tp.move_forward(0.05, True)
            else:
                hu.GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', 0.05, 20)
                success = True
            
            if not success:
                grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                grasp_status_msg = "Unable to move forward to grasp!"

        if Hao:
            """new routine"""
            hu.GuardedCloseHand(self.global_data)
        else:        
            """old routine"""
            if success:
                success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.pre_grasp_dof[1],grasp_msg.pre_grasp_dof[2], grasp_msg.pre_grasp_dof[3], grasp_msg.pre_grasp_dof[0]])


            if success:
                success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]])            


            if success:
                success, grasp_status_msg, joint_angles = tp.close_barrett()
            if not success:
                grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR
                        

        if success:
            selection = int(raw_input('Lift up (1) or not (0): '))
            if selection == 1:
                print 'lift up the object'
                success = tp.lift_arm(.05, True)
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                    grasp_status_msg = "Couldn't lift object"
            else:
                print 'not lift up the object'
            

        #Maybe decide if it has been successfully lifted here...
        if success:
            rospy.logwarn(grasp_status_msg)
        else:
            rospy.logfatal(grasp_status_msg)
        self.graspit_status_publisher.publish(grasp_status, grasp_status_msg)
        print grasp_status_msg
        return grasp_status, grasp_status_msg

    def point_within_table_cube(self, test_point):
        [min_corner_point , max_corner_point ] = self.table_cube 
        keys = ['x', 'y', 'z']
        for k in keys:
            t = getattr(test_point, k)
            min_test = getattr(min_corner_point, k)
            max_test = getattr(max_corner_point, k)
            if t < min_test or t > max_test:
                print 'Failed to be inside table in key %s - min - %f max - %f value %f'%(k, min_test, max_test, t)
                return False
        return True




if __name__ == '__main__':
    try:        
        rospy.init_node('graspit_message_robot_server')
        init_planner = rospy.get_param('init_planner', True)
        print "init planner value %d \n"%(init_planner)
        ge = GraspExecutor(init_planner = init_planner)
        if Hao:
            ae = AdjustExecutor(ge.global_data)
            se = ShakeExecutor(ge.global_data)
            me = MoveExecutor(ge.global_data)
        loop = rospy.Rate(10)
        #        ipshell = IPShellEmbed(banner = 'Dropping into IPython',
#                               exit_msg = 'Leaving Interpreter, back to program.')
#       ipshell(local_ns = locals())
        
        while not rospy.is_shutdown():
            ge.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException: pass

