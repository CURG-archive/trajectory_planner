"""
@package - model_rec_manager
@brief - calls model_rec and then manages the resulting models. Broadcast the pointclouds and TF

"""

import roslib; roslib.load_manifest( "trajectory_planner" )
import rospy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs

import tf
import tf.transformations
import tf_conversions.posemath as pm

import geometry_msgs
 
from tf import transformations as tr

import model_rec2, model_rec2.srv
import sensor_msgs, sensor_msgs.msg
import graspit_msgs.srv
import pdb
import visualization_msgs.msg
import point_cloud2
import geometry_msgs.msg
import std_msgs.msg


class ModelRecManager( object ):
    tf_listener = []
    tf_broadcaster = []
    class ModelManager( object ):
        def __init__(self, model_name, point_cloud_data, pose):
            self.model_name = model_name
            self.object_name = model_name
            self.point_cloud_data = point_cloud_data
            self.pose = pose
            self.bc = ModelRecManager.tf_broadcaster
            self.listener = ModelRecManager.tf_listener
            
        def __call__(self):
            tf_pose = pm.toTf(pm.fromMsg(self.pose))
            self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, "/camera_rgb_optical_frame")
            
        def get_dist(self):
            self.__call__()
            self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))
            (trans, rot) = self.listener.lookupTransform("/world",self.object_name, rospy.Time(0))
            return linalg.norm(trans)

        def __len__(self):
            return self.get_dist()

        def get_world_pose(self):
            self.__call__()
            self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))            
            return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world",self.object_name, rospy.Time(0))))
            
            
    def __init__(self, tf_listener = [], tf_broadcaster = []):
        if rospy.get_name() =='/unnamed':
            rospy.init_node('model_rec_node')
        self.__publish_target = True
        self.__publish_marker_array = True
        self.model_list = list()

        if not tf_listener:
            tf_listener = tf.TransformListener()
        if not tf_broadcaster:
            tf_broadcaster = tf.TransformBroadcaster()
            
        ModelRecManager.tf_listener = tf_listener
        ModelRecManager.tf_broadcaster = tf_broadcaster
        self.model_name_server = rospy.Service('/get_object_info', graspit_msgs.srv.GetObjectInfo, self.get_object_info)
        self.table_cube=[geometry_msgs.msg.Point(-0.7,-1.0,0), geometry_msgs.msg.Point(0,0,0)]
        

    def refresh(self):
        find_objects_srv = rospy.ServiceProxy('/recognize_objects', model_rec2.srv.FindObjects)
        resp = find_objects_srv()
        self.model_list = list()
        for i in range(len(resp.object_name)):
            #pdb.set_trace()
            special_names = ['snapple', 'krylon_spray', 'library_cup']
            if resp.object_name[i] in special_names:
                pose_modified = self.align_pose(resp.object_pose[i])
            else:
                pose_modified = resp.object_pose[i]
            self.model_list.append(self.ModelManager(resp.object_name[i],
                                                     resp.pointcloud[i],
                                                     pose_modified))
        for j in self.model_list:
             j.model_name = '/'+j.model_name
             j.point_cloud_data.header.frame_id=j.model_name

    def __call__(self):
        self.uniquify_object_names()
        if self.__publish_target:
            self.publish_target_pointcloud()
        if self.__publish_marker_array:
            self.publish_object_markers()

    def publish_target_pointcloud(self):
        self.model_list.sort(key=ModelRecManager.ModelManager.get_dist)
        x = self.model_list[0]
        print x.get_dist()
        tf_pose = pm.toTf(pm.fromMsg(x.pose))
        x.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/object", "/camera_rgb_optical_frame")
        x.listener.waitForTransform("/world", "/object", rospy.Time(0),rospy.Duration(5))
        x.point_cloud_data.header.frame_id = "/object"
        pub = rospy.Publisher('/object_pointcloud',sensor_msgs.msg.PointCloud2)        
        pub.publish(x.point_cloud_data)

        
    def publish_object_markers(self):
        marker_array = visualization_msgs.msg.MarkerArray()                    
        for model in self.model_list:
            marker = visualization_msgs.msg.Marker()
            marker.pose = model.get_world_pose()
            marker.header.frame_id='/world'
            marker.type = marker.POINTS
            marker.scale.x = .01
            marker.scale.y = .01
            marker.scale.z = .01
            marker.lifetime = rospy.Duration()
            point_generator = point_cloud2.read_points(model.point_cloud_data, None, True)
            marker.points = [geometry_msgs.msg.Point(point[0], point[1], point[2]) for point in point_generator]
            marker.colors = [std_msgs.msg.ColorRGBA(1,1,1,1) for point in marker.points]
            marker_array.markers.append(marker)
        pub = rospy.Publisher('/object_marker_array', visualization_msgs.msg.MarkerArray)
        pub.publish(marker_array) 
            
    def rebroadcast_object_tfs(self):
        for model in self.model_list:
            model()

    def get_model_names(self):
        return [model.model_name for model in self.model_list]

    def get_object_info(self, req):
        resp = graspit_msgs.srv.GetObjectInfoResponse
        for model in model_list:
            resp.object_info.append(graspit_msgs.msg.ObjectInfo(model.model_name, model.get_world_pose()))
        return resp
        
    def uniquify_object_names(self):
        object_names = {}
        for model in self.model_list:
            if model.model_name not in object_names.keys():
                object_names[model.model_name] = 0
            model.object_name = "%s_%i"%(model.model_name, object_names[model.model_name])
            object_names[model.model_name] = object_names[model.model_name] + 1
        


    def align_pose(self, pose):
        objectInCamera = pm.toMatrix(pm.fromMsg(pose))
        ModelRecManager.tf_listener.waitForTransform("/world", "/camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(5))
        cameraInWorld = pm.toMatrix(pm.fromTf(ModelRecManager.tf_listener.lookupTransform("/world", "/camera_rgb_optical_frame",rospy.Time(0))))
        objectInWorld = dot(cameraInWorld, objectInCamera)
        """45 degrees rotated around z axis"""
        objectInWorld[0:3,0:3] = mat([[0.7071,-0.7071,0],
                                      [0.7071,0.7071,0],
                                      [0,0,1]]);
        worldInCamera = linalg.inv(cameraInWorld)
        objectInCameraModified = dot(worldInCamera, objectInWorld)
        return pm.toMsg(pm.fromMatrix(objectInCameraModified))

def point_within_cube(test_point, min_corner_point, max_corner_point):
    keys = ['x','y','z']
    for k in keys:
        t = getattr(test_point, k)
        if t < getattr(min_corner_point, k) or t > getattr(max_corner_point, k):
            return False
    return True

                     
#      print x.listener.lookupTransform("/world","/object", rospy.Time(0))
    
