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
from adjust_message_robot_server import AdjustExecutor, ShakeExecutor, MoveExecutor
from time import time
from model_rec_manager import *
import HaoUtil as hu
import grasp_analyzer

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
    
        self.refresh_models_listener = rospy.Subscriber("/graspit/refresh_models", Empty, self.refresh_model_list)

        self.reload_models_listener = rospy.Subscriber("/graspit/reload_models", Empty, self.reload_model_list)
        
        
        self.global_data = tp.SetupStaubliEnv(True, init_planner)
        
        
        
        self.model_manager = ModelRecManager(self.global_data.listener)
        self.graspit_status_publisher = rospy.Publisher("/graspit/status", graspit_msgs.msg.GraspStatus)

        self.graspit_scene_publisher = rospy.Publisher("/graspit/scene_info", graspit_msgs.msg.SceneInfo)
        self.remove_object_publisher = rospy.Publisher("/graspit/remove_objects", String)

        


        self.last_grasp_time = 0
        self.table_cube=[geometry_msgs.msg.Point(-0.7,0,-0.02), geometry_msgs.msg.Point(0.2,1,1)]
        self.grasp_analyzer = grasp_analyzer.GraspAnalyzer(self.global_data)
        if bool(rospy.get_param('reload_model_rec',0)):
            self.reload_model_list([])
            

    def refresh_model_list(self, empty_msg):        
        self.model_manager.refresh()
        self.model_manager()
        
        self.remove_object_publisher.publish('ALL')
        self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()

    def reload_model_list(self, empty_msg):
        self.model_manager.read()
        self.model_manager()
        
        self.remove_object_publisher.publish('ALL')
        self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()


    def publish_table_models(self):
        """
        @brief - Publishes only the objects that are in a prism above the table to GraspIt        
        """

        #Republish all of the object pose TFs
        self.model_manager()

        #get a list of models in that cube above the table
        table_models = [model for model in self.model_manager.model_list
                        if self.point_within_table_cube(model.get_world_pose().position)]

        #Print a list of rejected models to the terminal
        print '\n'.join(['Model rejected %s'%(model.object_name)
                         for model in self.model_manager.model_list if model not in table_models])
        
        #For each model in the valid model list, add the model to the object list
        #and publish it to GraspIt!
        #FIXME -- needs to use scene message
        object_list = []
        scene_msg = graspit_msgs.msg.SceneInfo()
        for model in table_models:
            model()
            
            p = model.get_world_pose()
            print "Model name: %s"%(model.object_name)
            print p
            object_list.append(graspit_msgs.msg.ObjectInfo(
                                                 model.object_name, model.model_name, p))
        scene_msg.objects = object_list
        self.graspit_scene_publisher.publish(scene_msg)


    def add_object_to_planner(self, model):
        """
        @brief - Adds a model_rec_manager model to openrave
        """
        tp.add_object_to_planner(self.global_data,
                                 model.object_name.strip('/'),
                                 file_name_dict[model.model_name.strip('/')])




    def add_all_objects_to_planner(self):
        """
        @brief - Adds all of the models in the model_rec_manager to openrave
        """
        for model in self.model_manager.model_list:
            self.add_object_to_planner(model)

    def remove_all_objects_from_planner(self):
        """
        @brief - Clears all models from openrave except the experiment table and the staubli arm

        FIXME - Add additional obstacles for camera post and computers around robot
        and some way of annotating that they shouldn't be removed.
        """
        body_list = self.global_data.or_env.GetBodies()
        ignored_body_list = ['StaubliRobot', 'table']
        for body in body_list:
            if body.GetName() not in ignored_body_list:
                self.global_data.or_env.Remove(body)
                del body
    
        

    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to grasp the object and lift it

        First moves the arm to pregrasp the object, then attempts to grasp and lift it.
        The grasp and lifting phase is currently completely open loop
        
        """
        try:
            #Lock the openrave environment - it will restore the robot and collision state
            #to it's previous state when it finishes automagically.
            
            with self.global_data.or_env:

                #Reject multiple grasp messages sent too fast. They are likely a mistake
                #FIXME - We should probably just set the queue size to 1.
                if (time() - self.last_grasp_time) < 30:
                    return [], []
                self.last_grasp_time = time()

                print grasp_msg
                grasp_status = graspit_msgs.msg.GraspStatus.SUCCESS
                grasp_status_msg = "grasp_succeeded"
                success = 1
                
                # Send the robot to its home position if it is actually running
                #and not currently there
                if self.global_data.robot_running and not tp.is_home():
                    print 'go home'
                    tp.go_home(self.global_data)
                    #This sometimes fails near the end of the trajectory because the last few
                    #steps of the trajectory are slow because our blending parameters
                    #are probably wrong. It is basically in the home position, so we
                    #don't check for success in reaching the home position because meaningful
                    #failures are rare.


                #Open the hand - Leaves spread angle unchanged
                if success and self.global_data.robot_running:
                    success, grasp_status_msg, positions = tp.open_barrett()
                    if not success:
                        grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

                
                if success:
                    #Pregrasp the object
                    
                    #Convert the grasp message to a openrave transform
                    grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
                    grasp_tran[0:3,3] /=1000 #mm to meters

                    #Move the hand to the pregrasp spread angle - Update openrave's robot pose
                    if self.global_data.robot_running:
                        tp.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
                        tp.update_robot(self.global_data.or_env.GetRobots()[0])

                        print 'pre-grasp'

                    #Publish the object TFs
                    self.model_manager()
                    print 'after model_manager()'

                    #Calculate a trajectory to the pregrasp pose and move to it if necessary
                    success, final_tran, dof_list, j = tp.pregrasp_object(self.global_data, grasp_msg.object_name,  False, grasp_tran, run_it = self.global_data.robot_running)
                    print 'after pre-grasp'

                    #Failures shouldn't happen if grasp analysis is being used, so that's wierd.
                    if not success:
                        pdb.set_trace()
                        
                    #Make sure that the openrave environment is up to date in pregrasp pose
                    if self.global_data.robot_running:
                        tp.update_robot(self.global_data.or_env.GetRobots()[0])
                    else:
                        #If the robot isn't actually running, just update the openrave visualization
                        #so that the user can see what was planned.
                        if success:
                            print "setting robot joints"
                            print j
                            self.global_data.or_env.GetRobots()[0].SetActiveDOFs(range(6))
                            self.global_data.or_env.GetRobots()[0].SetActiveDOFValues(dof_list[-1])
                            self.global_data.or_env.GetRobots()[0].SetActiveDOFs(range(6,10))
                            self.global_data.or_env.GetRobots()[0].SetActiveDOFValues(list(grasp_msg.pre_grasp_dof[1:]) + [grasp_msg.pre_grasp_dof[0]])                    
                            self.global_data.or_env.GetRobots()[0].SetActiveDOFs(range(6))
                            self.global_data.or_env.UpdatePublishedBodies()
                            pdb.set_trace()
                            success = False

                    #If the pregrasp was unreachable, record the type of issue
                    if not success:
                        #If there is no joint list, the pregrasp transform had no inverse
                        #kinematics solution, probably. 
                        if not j:
                            grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                            grasp_status_msg = "Pregrasp tran Out of range!"
                        #Otherwise, something even wierder happened.
                        else:
                            grasp_status = graspit_msgs.msg.GraspStatus.FAILED
                            grasp_status_msg = "Unknown planner failure!"


            #Move from the pregrasp to the final grasp pose in a straight line along the approach
            #direction
            if success:
                if not Hao:
                    #Move forward open loop
                    success = tp.move_forward(0.05, True)
                else:
                    #Move forward using force transducer and palm contact sensors to detect any
                    #collisions
                    hu.GuardedMoveUntilHit(self.global_data, array([0,0,1]), 'PALM', 0.05, 20)
                    success = True

                if not success:
                    #If we failed to move forward - which might happen because
                    #we do not test if the final grasp pose is reachable along
                    #a straight line trajectory because the joint range limits
                    #set on the robot are not actually exactly the same as those
                    #in openrave and it usually doesn't cause any problems
                    #in general trajectory testing. 
                    grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                    grasp_status_msg = "Unable to move forward to grasp!"

            if Hao:
                """new routine"""
                #Close the hand using the force torque and contact sensors to detect contacts and
                #stop the fingers
                hu.GuardedCloseHand(self.global_data)
            else:        
                """old routine"""
                if success:
                    #Just close the hand open loop, leaving the spread angle where it is now. 
                    success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.pre_grasp_dof[1],grasp_msg.pre_grasp_dof[2], grasp_msg.pre_grasp_dof[3], grasp_msg.pre_grasp_dof[0]])


                if success:
                    #Close the hand completely until the motors stall or they hit
                    #the final grasp DOFS
                    success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]])            


                if success:
                    #Now close the hand completely until the motors stall.
                    success, grasp_status_msg, joint_angles = tp.close_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            #Now wait for user input on whether or not to lift the object
            if success:
                selection = int(raw_input('Lift up (1) or not (0): '))
                if selection == 1:
                    print 'lift up the object'
                    #Lift the object using the staubli's straight line path planner
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
            #Tell graspit whether the grasp succeeded
            self.graspit_status_publisher.publish(grasp_status, grasp_status_msg, -1)
            
            print grasp_status_msg

            return grasp_status, grasp_status_msg

        except Exception as e:
            #print any exceptions and drop in to the debugger.
            import traceback
            print traceback.format_exc()
            pdb.set_trace()



    def point_within_table_cube(self, test_point):
        """
        @brief - Test whether a point is within a cube defined by its
        lower left and upper right corners. The corners are stored in the table_cube
        member. 

        FIXME - This implementation is likely slow and dumb, but it works for me. 

        @param test_point - The point to test
        """
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
        run_interactive = rospy.get_param('INTERACTIVE_AUTOMATED_CONTROL', 0)
        if run_interactive:
            ipshell = IPShellEmbed(banner = 'Dropping into IPython',
                                   exit_msg = 'Leaving Interpreter, back to program.')
            ipshell(local_ns = locals())
        
        while not rospy.is_shutdown():
            ge.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException: pass

