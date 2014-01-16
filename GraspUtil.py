# Copyright (c) 2010-2011 REARM Team 
#   Author: Aravind Sundaresan <aravind@ai.sri.com>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import re 
import os
import time
import math
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs

import pdb
import openravepy as orpy

import util.str2num 
from util.TSR import TSR, TSRChain
from util.TransformMatrix import Serialize1DMatrix

import TSRUtil

import roslib; roslib.load_manifest( "trajectory_planner" )
import rospy
import geometry_msgs
from tf import transformations as tr
import tf_conversions.posemath as pm
import graspit_msgs
import copy

# set logger levels for this function 
logdebug = rospy.logdebug
loginfo = rospy.loginfo
logwarn = rospy.logwarn
logerr = rospy.logerr

'''
Get list of darpa objects in the scene 
'''

def GetDarpaObjectsInScene( global_data, whichmanip ):
    actionableobjs = []
    bodies = global_data.orEnv.GetBodies()
    for body in bodies:
        if 'darpa' in body.GetName():
            actionableobjs.append( body )
    
    return actionableobjs

'''
Get list of darpa object names in the scene 
Exclude the names in the list 
'''

def GetDarpaObjectNamesInScene( global_data, whichmanip, excludeprefixes=[] ):
    objnames = []
    bodies = global_data.orEnv.GetBodies()
    for body in bodies:
        if 'darpa' in body.GetName():
            objnames.append( body.GetName() )

    excludenames = []
    for objname in objnames:
        for excludename in excludeprefixes:
            if excludename in objname:
                excludenames.append( objname )
                break

    objnames = list( set( objnames ) - set( excludenames ) )
    
    return objnames

''' 
' Strip numbers from the end of the name 
''' 

def StripIndex( objName ):
    objKey = objName
    for i in range( 1 ):
        objKey = re.sub( '[ 0-9]', '', objKey )
    return objKey

''' 
' Convert t and q to Pose    
''' 

def ToPose( t, q ):
    orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] ) 
    position = geometry_msgs.msg.Point( t[0], t[1], t[2] ) 
    return geometry_msgs.msg.Pose( position, orientation )

''' 
' Compute geometry_msgs::Pose from 4x4 transform 
'   G is a 4x4 matrix which is [R t; 0 0 0 1]
'   
''' 

def TransformToPose( G ):
    t = array( G )[0:3,3]
    q = tr.quaternion_from_matrix( G )
    orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] ) 
    position = geometry_msgs.msg.Point( t[0], t[1], t[2] ) 
    pose = geometry_msgs.msg.Pose( position, orientation )
    return pose

''' 
' Compute 4x4 transform from geometry_msgs::Pose 
'   G is a 4x4 matrix which is [R t; 0 0 0 1]
'   
''' 

def PoseToTransform( pose ):
    q = pose.orientation
    t = pose.position

    G = tr.quaternion_matrix( array( [ q.x, q.y, q.z, q.w ] ) )
    G[0:3,3] = array( [ t.x, t.y, t.z ] )
    
    return G

''' 
' Return the x,y, and z-axis of the transform
''' 

def MatrixToAxes( G ):
    G = array( G )
    xaxis = G[0:3,0]
    yaxis = G[0:3,1]
    zaxis = G[0:3,2]
    t = G[0:3,3]
    return xaxis, yaxis, zaxis, t

''' 
' Compute 4x4 array from x,y,z axis and translation
''' 

def AxesToMatrix( xaxis, yaxis, zaxis, t = array([0,0,0]) ):
    G = eye( 4 )
    G[0:3,0] = xaxis 
    G[0:3,1] = yaxis 
    G[0:3,2] = zaxis 
    G[0:3,3] = t
    return G

''' 
' Convert 4x4 transform into a string 
''' 

def PoseToStr( pose ):
    t = pose.position
    q = pose.orientation
    fstr = "[ [%.2f %.2f %.2f]  [%.2f %.2f %.2f %.2f] ]" % ( t.x, t.y, t.z, 
            q.x, q.y, q.z, q.w )
    return fstr


''' 
' Convert 4x4 transform into a string 
''' 

def TranToStr( G ):
    pose = TransformToPose( G )
    t = pose.position
    q = pose.orientation
    fstr = "[ [%.2f %.2f %.2f]  [%.2f %.2f %.2f %.2f] ]" % ( t.x, t.y, t.z, 
            q.x, q.y, q.z, q.w )
    return fstr


'''
' Compute 4x4 matrix from 3x3 and 3x1 
'''

def Rt2G( R, t = mat([0,0,0]) ):
    Rt = hstack( ( R, mat(t).reshape( 3, 1 ) ) )
    return vstack( ( Rt, array( [0, 0, 0, 1] ).reshape( 1, 4 ) ) )

'''
' Get 3x3 matrix and 3x1 from 4x4 
'''

def G2Rt( G ):
    R = G[0:3,0:3]
    t = G[0:3,3]
    return R,t

'''
' Get 3x3 matrix and 3x1 from 4x4 
'''

def G2qt( G ):
    q = tr.quaternion_from_matrix( G )
    t = G[0:3,3]
    return q,t

''' 
' Get object pose from environment 
''' 

def GetObjectPose( global_data, objName, debug=False ):
    success, tran = GetObjectTransform( global_data, objName, debug )
    return success, TransformToPose(tran)

''' 
' Get object transform from environment 
''' 

def GetObjectTransform( global_data, objName, debug=False ):
    success = 0
    tran = eye( 4 )
    if debug:
        logdebug( "  Bodies in the environment" )
        #pdb.set_trace()
    for body in global_data.orEnv.GetBodies():
        gotcha = ' '
        if objName in body.GetName():
        #if 'darpadrill' in body.GetName():
            tran = body.GetTransform()
            gotcha = '*'
            success = 1
        if debug:
            loginfo( " (%s) found: %s (%s)"%( objName, gotcha, body.GetName()) )
    
    if not success: 
        logwarn( '  Object %s does not exist' %( objName ) )

    return success, tran

''' 
' Plot openbhand for the list of poses
'''

def PlotTSRGoals( global_data, tsrList ):
    handName = 'openbhand'
    handFile = "%s/objects/misc/%s.kinbody.xml" % ( global_data.openrave_models_path, handName )
    RemoveHandfromEnv( global_data )

    nList = len( tsrList )
    loginfo( "Plotting %d-%d of %d" % ( 1, nList, nList ) )
    for i in range( nList ):
        T0_g = TSRUtil.E2G( tsrList[i].T() )
        T = array( ( T0_g[0:3][:,0:4] ) )
        target = global_data.orEnv.ReadKinBodyXMLFile( handFile )
        target.SetName( '%s%d' % ( handName, i ) )
        target.SetTransform( T )
        global_data.orEnv.AddKinBody( target )

    return 1


''' 
' Plot openbhand for the list of poses
'''

def PlotGraspitGoals( global_data, graspList ):
    handName = 'openbhand'
    handFile = "%s/objects/misc/%s.kinbody.xml" % ( global_data.openrave_models_path, handName )
    RemoveHandfromEnv( global_data )

    nList = len( graspList )
    loginfo( "Plotting %d-%d of %d" % ( 1, nList, nList ) )
    for i in range( nList ):
        T0_g = PoseToTransform( graspList[i].pre_grasp_pose )
        T = array( ( T0_g[0:3][:,0:4] ) )
        target = global_data.orEnv.ReadKinBodyXMLFile( handFile )
        target.SetName( '%s%d' % ( handName, i ) )
        target.SetTransform( T )
        global_data.orEnv.AddKinBody( target )

    return 1


''' 
Remove the targets from the environment
'''

def RemoveObjects( global_data, targets ):
    for target in targets.split():
        for body in global_data.orEnv.GetBodies():
            if target in body.GetName():
                global_data.orEnv.Remove( body )
                break


''' 
Remove hands from the environment
'''

def RemoveHandfromEnv( global_data ):
    handName = 'openbhand'

    for body in global_data.orEnv.GetBodies():
        if handName in body.GetName():
            global_data.orEnv.Remove( body )


''' 
Add padded object to environment
'''

def AddPaddedObject( global_data, objName, axisFlag = False ):
    fileName = '%s/objects/misc/padded_%s.kinbody.xml' % (
            global_data.openrave_models_path, StripIndex( objName ) )

    if not os.path.exists( fileName ):
        reason = "File %s does not exist" % fileName
        return 0, reason
    
    success, objPose = GetObjectPose( global_data, objName, True )
    if success == 0:
        reason = "Cannot get pose of %s from OpenRAVE" % objName
        return 0, reason

    try:
        target = global_data.orEnv.ReadKinBodyXMLFile( fileName )
        target.SetTransform( PoseToTransform( objPose ) )
        global_data.orEnv.AddKinBody( target )
    except orpy.openrave_exception, err:
        reason = "Error: %s (cannot load %s)" % ( err, fileName )
        return 0, reason

    if axisFlag:
        fileName = '%s/objects/misc/darpaaxis.kinbody.xml' % ( global_data.openrave_models_path )
        AddObject( global_data, fileName, objPose )

    return 1, ""


''' 
Add padded object to environment
'''

def AddObject( global_data, fileName, objPose ):
    if not os.path.exists( fileName ):
        logwarn( "File %s does not exist" % fileName )
        return 0

    try:
        target = global_data.orEnv.ReadKinBodyXMLFile( fileName )
        target.SetTransform( PoseToTransform( objPose ) )
        global_data.orEnv.AddKinBody( target )
    except orpy.openrave_exception, err:
        logerr( "Error: %s (cannot load %s)" % ( err, fileName ) )
        return 0

    return 1


''' 
' Remove padded object from environment
'''

def RemovePaddedObjects( global_data ):
    paddedObjectName = 'padded_darpa'

    for body in global_data.orEnv.GetBodies():
        if paddedObjectName in body.GetName():
            global_data.orEnv.Remove( body )
        if 'darpaaxis' in body.GetName():
            global_data.orEnv.Remove( body )


'''
Get the distance between two poses. define the distance as angle between poses) 
+ difference in offset)
'''
def interPoseDistance(p1, p2):
    r = p1.I*p2;
    rMsg  = pm.toMsg(pm.fromMatrix(r))
    angleDistance = abs(math.acos(rMsg.orientation.w))
    positionDistance = linalg.norm(p1[0:3,3] - p2[0:3,3])
    return angleDistance + positionDistance

#'''
#'Helper function to transform graspit pose messages to the world frame
#'Graspit defines a different coordinate system from everyone else.  This will eventually move in to
#'the graspit server in some for, but for now it lives here.
#
#XXX replace this with TSRUtil.G2E( GraspUtil.PoseToTransform( graspitPose ) )
#'''
#def convertGraspitPoseToWorldPose(graspitPose):
#    tpreRot = pm.fromMatrix(mat([[0, -1, 0, 0 ],[1, 0, 0, 0],[0, 0, 1, 0],[0,0,0,1]]))
#    backupTran = pm.fromMatrix(mat([[1, 0, 0, 0],[0, 1, 0, 0 ],[ 0, 0, 1, -.135],[0 ,0, 0, 1]]))
#    intermediatePose = pm.fromMsg(graspitPose)*tpreRot
#    return pm.toMatrix(intermediatePose*backupTran)

'''
Compute the closest grasp to robot position 
'''
def GetClosestGraspToRobotPosition(whichmanip, global_data, graspList):
    # Get the Z direction of the palm    
    manip = global_data.robot.GetManipulators()[whichmanip]
    T0_ee = mat(manip.GetEndEffectorTransform())
    distList = list()
    for i in range(len(graspList)):
        T0_g_i = PoseToTransform( graspList[i].pre_grasp_pose )
        T0_e_i = TSRUtil.G2E( T0_g_i )
        distList.append(interPoseDistance(T0_ee, T0_e_i))
    # compute index of minimum value 
    imin = distList.index(min(distList))
    return graspList[imin]

    
''' 
Convert Graspit grasps to TSRs
Return items 
+ TSR planner string  
+ list of transforms of goal poses of graspit hand
'''

def GraspitListToTSRList( grasps, numGrasps=100 ):
    success = 1
    tsrList = []
    numGrasps = min( len( grasps ), numGrasps )
    Bw_T = 0.01
    Bw_R = 2.5 * pi / 180
    for i in range( numGrasps ):
        T0_g = PoseToTransform( grasps[i].pre_grasp_pose )
        tsr = TSRUtil.GraspitToTSRSimple( T0_g, Bw_T, Bw_R )
        tsrList.append( copy.deepcopy( tsr ) )

    return tsrList

'''
Convert tsrList to TSR string 
Specify a non-empty Bw_notilt with the appropriate bounds for additional
constraints. If Bw_notilt = [], then no constraints will be added. 

'''
def TSRListToTSRString( global_data, whichmanip, tsrList, Bw_notilt, numTSRs=100, bStartSampling = 0, bDestinationSampling = 1):
    T0_gList = []
    TSRChainList = []
    numTSRs = min( len( tsrList ), numTSRs )
    if numTSRs == 0:
        return ""

    for i in range( numTSRs ):
        tempTSR = util.TSR.TSR()
        tempTSR.manipindex = whichmanip
        tempTSR.T0_w = tsrList[i].T0_w
        tempTSR.Tw_e = tsrList[i].Tw_e
        tempTSR.Bw = tsrList[i].Bw
        # Construct TSR for i^th grasp
        tsrChain = TSRChain(bStartSampling, bDestinationSampling, 0)
        tsrChain.SetManipIndex( whichmanip )
        tsrChain.insertTSR( tempTSR )
        TSRChainList.append( tsrChain )

        T0_g = TSRUtil.E2G( tempTSR.T0_w * tempTSR.Tw_e )
        T0_gList.append( T0_g )
        
    TSRstring = ""
    TSRstring = TSRstring + ' %s'%(' '.join(' %s'%(tsrchain.Serialize()) for tsrchain in TSRChainList))

    doNotTilt = len( Bw_notilt ) > 0
    if doNotTilt:
        logwarn( "Constraining TSR to not tilt" )
        #get the no-tilting TSR
        constraintTSR = util.TSR.TSR()
        constraintTSR.T0_w = tsrList[i].T0_w
        constraintTSR.Tw_e = tsrList[i].Tw_e
        #constraintTSR.Bw = mat([-100, 100, -100, 100, -100, 100, -0.1,0.1, -0.1,0.1, -pi,pi])
        constraintTSR.Bw = Bw_notilt
        constraintTSR.manipindex = whichmanip
        constraintTSRChain = TSRChain(bStartSampling,bDestinationSampling,1)
        constraintTSRChain.insertTSR(constraintTSR)
        constraintTSR.SetManipIndex(whichmanip)
        TSRstring = '%s %s' % ( TSRstring, constraintTSRChain.Serialize() )

    TSRstring = TSRstring + " psample %f" % global_data.psample

    return TSRstring

'''
Get Graspit grasps by calling service 
Returns graspList
'''

def GetGraspitGraspList( objectName, objectPose, tablePose ):
    success = 0
    graspList = []
    loginfo( "Calling Graspit! service for %s" %( objectName ) )
    
    xa_o, ya_o, za_o, t_o = MatrixToAxes( PoseToTransform( objectPose ) )
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
    angle = math.acos( abs( dot( za_o, za_t ) ) ) * 180 / math.pi

    grasp_constraints = 0

    if angle > 45:
        grasp_constraints = 1

    try:
        GraspitSrv = rospy.ServiceProxy( 'grasp_plan', PlanGraspService )
        res = GraspitSrv( objectName, objectPose, tablePose, grasp_constraints )
    except rospy.ServiceException, err:
        logerr( "  Service call failed: %s" % err )
        return success, graspList, "Graspit service call failed"

    success = 1
    graspList = res.grasps
    ngrasps = len( graspList )
    loginfo( "Graspit returned %d grasps " %( ngrasps ) )

    return success, graspList, ""

'''
Get generic grasps by assuming pipelike structure 
This function is a plugin replacement for GetGraspitGraspList()
This is not applicable for all objects
'''

def GetGenericGraspList( objectName, objectPose, tablePose ):
    success = 1
    loginfo( "Generating generic grasp for  %s" %( objectName ) )

    # if horizontal must be a minmum of 0.14 from the table 

    pre_backoff_dist = -0.14
    fin_backoff_dist = -0.06

    G_pre = mat( Rt2G( eye( 3 ), array( [0.,0.,pre_backoff_dist] ) ) )
    G_fin = mat( Rt2G( eye( 3 ), array( [0.,0.,fin_backoff_dist] ) ) )
    p = objectPose.position
    t_obj = mat( [p.x, p.y, p.z] )
    grasp = graspit_msgs.msg.Grasp()
    grasp.epsilon_quality = 0.1
    grasp.volume_quality = 0.1
    grasp.pre_grasp_dof = array( [0,0,0,0] )
    grasp.final_grasp_dof = array( [0,0,0,0] )

    graspList = []
    xa_t, ya_t, za_t, t_t = MatrixToAxes( PoseToTransform( tablePose ) )
    xa_o, ya_o, za_o, t_o = MatrixToAxes( PoseToTransform( objectPose ) )
    upright = dot( za_t, za_o )

    if upright > 0.75:
        loginfo( "  Object is vertical"  )
        Rx = tr.rotation_matrix( pi/2, array( [0, 1., 0] ) )
        for an in range(0,360,45):
            th = an * pi / 180
            # XXX FIXME Is assumption that R_obj = eye(3) should be explicit
            Rz = tr.rotation_matrix( th, array( [0, 0, 1.] ) )
            Rz[0:3,3] = t_obj
            G0 = mat( Rz ) * mat( Rx )
            G1 = mat( G0 ) * mat( G_pre )
            G2 = mat( G0 ) * mat( G_fin )

            grasp.pre_grasp_pose = TransformToPose( G1 )
            grasp.final_grasp_pose = TransformToPose( G2 )
            grasp.epsilon_quality = 0.1
            graspList.append( copy.deepcopy( grasp ) )
            zStr = "%.2f %.2f %.2f" % ( G1[0,2], G1[1,2], G1[2,2] )
            loginfo( "  %3d: %s, zaxis=%s" % ( an, PoseToStr( grasp.pre_grasp_pose ), zStr ) )
    else:
        loginfo( "  Object is horizontal"  )
        za = -za_t
        xa = za_o - dot( za_o, za ) * za
        xa = xa / sqrt( dot( xa, xa ) )
        ya = cross( za, xa )
        
        tstep = arange(-2,3) * 0.02
        for i in range( len( tstep ) ):
            #t = t_o + tstep[i] * za_o + tz * za_t
            G0 = AxesToMatrix( xa, ya, za, t_o )
            G1 = mat( G0 ) * mat( G_pre )
            G2 = mat( G0 ) * mat( G_fin )
            grasp.epsilon_quality = 0.1
            grasp.pre_grasp_pose = TransformToPose( G1 )
            grasp.final_grasp_pose = TransformToPose( G2 )
            graspList.append( copy.deepcopy( grasp ) )
            zStr = "%.2f %.2f %.2f" % ( G1[0,2], G1[1,2], G1[2,2] )
            loginfo( "  %3d: %s, zaxis=%s" % ( i, PoseToStr( grasp.pre_grasp_pose ), zStr ) )

    loginfo( "Generic grasps: %d grasps" %( len( graspList ) ) )

    return success, graspList, "Success"

'''
Get trigger grasps for darpadrill, darpaflashlight
This function is a plugin replacement for GetGraspitGraspList()
This is not applicable for all objects
'''

def GetManipGraspList( objectName, objectPose, tablePose, actionName ):
    loginfo( "Generating %s grasp for %s" %( actionName, objectName ) )
    graspList = []

    grasp = graspit_msgs.msg.Grasp()
    T0_o = PoseToTransform( objectPose )
    To_g_list = []
    if "darpadrill" in objectName:
        # Rx1 has 2 fingers along +ve y-axis
        # Rx2 has 2 fingers along -ve y-axis
        Rx1 = tr.rotation_matrix( -pi/2, array( [0, 1., 0] ) )
        Rx2 = tr.rotation_matrix( pi/2, array( [0, 1., 0] ) )
        for th in arange( 15, 75, 15) * pi/180:
            Rz1 = tr.rotation_matrix( th, array( [0, 0, 1.] ) )
            To_g = mat( Rz1 ) * mat( Rx1 )
            To_g_list.append( copy.deepcopy( To_g ) )
            Rz2 = tr.rotation_matrix( pi - th, array( [0, 0, 1.] ) )
            To_g = mat( Rz2 ) * mat( Rx2 )
            To_g_list.append( copy.deepcopy( To_g ) )
        
        G_fin = mat( Rt2G( eye( 3 ), array( [0.,0., -0.07] ) ) )
        G_pre = mat( Rt2G( eye( 3 ), array( [0.,0., -0.14] ) ) )
        G_tra = mat( Rt2G( eye( 3 ), array( [0.,0.,  0.00] ) ) )
        for To_g in To_g_list:
            grasp.pre_grasp_pose = TransformToPose( G_tra *T0_o * To_g * G_pre )
            grasp.final_grasp_pose = TransformToPose( T0_o * To_g * G_fin )
            grasp.epsilon_quality = 0.1
            grasp.volume_quality = 0.1
            graspList.append( copy.deepcopy( grasp ) )
    else:
        success = 0
        reason = "No trigger grasps for object %s" % objectName
        return success, graspList, reason

    loginfo( "Trigger grasps: %d grasps" %( len( graspList ) ) )

    success = 1
    reason = "Done"
    return success, graspList, reason

'''
Print TSR list 
'''
def PrintTSRList( tsrList, nPoses2=5 ):
    nPoses = len( tsrList ) 
    nPoses2 = min( nPoses2, nPoses )
    loginfo( "  TSR poses (%d-%d of %d)" %( 1, nPoses2, nPoses ) )
    for i in range( nPoses2 ):
        G = tsrList[i].Tw_e
        zStr = "%.2f %.2f %.2f" % ( G[0,2], G[1,2], G[2,2] )
        loginfo( "    %2d. Tw_e = %s, z-axis = %s" %( i + 1, TranToStr( G ), zStr ) )
        G = tsrList[i].T0_w * tsrList[i].Tw_e
        zStr = "%.2f %.2f %.2f" % ( G[0,2], G[1,2], G[2,2] )
        loginfo( "        T0_e = %s, z-axis = %s" %( TranToStr( G ), zStr ) )

    if nPoses2 < nPoses:
        loginfo( "... and %d more" % ( nPoses - nPoses2 ) )
    else:
        loginfo( "---" )

'''
Print Grasp list 
'''
def PrintGraspList( graspList, nGrasps2=5 ):
    nGrasps = len( graspList )
    nGrasps2 = min( nGrasps, nGrasps2 )
    loginfo( "  Grasp poses (%d-%d of %d)" % ( 1, nGrasps2, nGrasps ) )
    for i in range( nGrasps2 ):
        loginfo( "    %2d. %s" %( i, PoseToStr( graspList[i].pre_grasp_pose ) ))

    if nGrasps2 < nGrasps:
        loginfo( "... and %d more" %( nGrasps - nGrasps2 ) )
    else:
        loginfo( "---" )

    return 


'''
Get list of TSRs for placing object on target  

  T_target: Pose of the target where the xy plane is aligned with the
      surface and the z-axis pointing upwards. 
  T_ee    : Pose of the end-effector 

  The TSRs are computed so that the palm is 
  - directly above the target, if facing the table
  - slightly behind the target (backoff) if almost perpendicular to the table 
'''

def GetPlaceOnTSRList( T_target, T_ee, Bw_T, Bw_R, avalues = [0] ):
    T0_w = T_target 
    T0_p = TSRUtil.E2P( T_ee )
    Tw_p = linalg.inv( T0_w ) * T0_p 
    Tw_p[0,3] = 0
    Tw_p[1,3] = 0

    # backoff distance from center of target 
    backoff = -0.03

    # determine if EE is perpendicular or parallel to target
    [x_t, y_t, z_t, t_t] = MatrixToAxes( T_target )
    [x_e, y_e, z_e, t_e] = MatrixToAxes( T_ee )
    t_backoff = array( [0, 0, backoff * ( 1 - fabs( dot( z_t, z_e ) ) )] )
    G_backoff = mat( Rt2G( eye( 3 ), t_backoff) )

    tsrList = []
    for i in range( len( avalues ) ):
        th = avalues[i] * pi / 180
        Rz = mat( tr.rotation_matrix( th, array( [0, 0, 1.] ) ) )
        Tw_p_i = Rz * Tw_p * G_backoff
        T0_p_i = T0_w * Tw_p_i
        tsr_i = TSRUtil.PalmToTSR( T0_p_i, T0_w, Bw_T, Bw_R )

        tsrList.append( copy.deepcopy( tsr_i ) )

    return tsrList

'''
'''

def GetObjectPosesForGrasping(global_data,  targets):
    success = 1
    tablePose = []
    objectPose = [] 
    loginfo( "Planning grasps for %s" % (targets)  )

    targetObjects = targets.split()
    if len( targetObjects ) > 1:
        loginfo( "  Selecting first object in list (%s)" % (targetObjects[0]) )
    elif len( targetObjects ) == 0:
        logerr( "  Must specify at least one target" )
        success = 0

    # Get object pose and name (minux index)
    objName = targetObjects[0]
    tableName = 'darpatable'
    if success:
        success1,objPose = GetObjectPose( global_data, objName )
        if success1 == 0:
            logerr( "Cannot get pose of %s" % objName )
            success = 0
        success2,tablePose = GetObjectPose( global_data, tableName )
        if success2 == 0:
            logerr( "Cannot get pose of %s" % tableName )
            success = 0
    return success, objPose, tablePose, objName




'''
*** Top level function ***

Get Graspit TSRString for the given target


'''

def GetGraspItTSRString( global_data, whichmanip, targets, graspListModifier=[] ):
    useGraspIt = True 
    # maximum number of grasps to use for constructing TSR 
    success = 0
    tsrList = []
    plannerStr = ''
    activeDofs = global_data.armdofs[whichmanip]
    success, objPose, tablePose, objName = GetObjectPosesForGrasping(global_data, targets)
    if success:       
        doIKCheck = True
        if useGraspIt:
            # get graspList from graspit 
            success, graspList, rstr = GetGraspitGraspList( StripIndex(objName), objPose, tablePose )
            if graspListModifier:
                success, graspList = graspListModifier(graspList)
                
        else:
            # get generic graspList 
            success, graspList, rstr = GetGenericGraspList( objName, objPose, tablePose )
            
        if doIKCheck:
            tsrList, plannerStr = GraspitListToIKTSRString( global_data, whichmanip, graspList )
            # If list of grasps is zero, set pre_grasp to final_grasp and try
            # again
            if len( tsrList ) == 0:
                logwarn( " " )
                logwarn( "*** Setting pre_grasp to final_grasp ***" )
                for grasp in graspList:
                    grasp.pre_grasp_pose = grasp.final_grasp_pose

                tsrList, plannerStr = GraspitListToIKTSRString( global_data, whichmanip, graspList )

                # If list of grasps is still zero, use normal TSR string 
                if len( tsrList ) == 0:
                    logwarn( " " )
                    logwarn( "*** Not using joint goals ***" )
                    maxGrasps = 50
                    tsrList = GraspitListToTSRList( graspList, maxGrasps )
                    Bw_notilt = []
                    plannerStr = TSRListToTSRString( global_data, whichmanip, tsrList, Bw_notilt )

        else:
            maxGrasps = 50
            tsrList = GraspitListToTSRList( graspList, maxGrasps )
            Bw_notilt = []
            plannerStr = TSRListToTSRString( global_data, whichmanip, tsrList, Bw_notilt )

    return success, plannerStr, activeDofs, tsrList, graspList


'''
*** Top level function ***

Get Place On TSR String for given targets 
Note: The no-tilt constraint also specifies that the z-axis is lower bounded by
the initial position of the TSR (minus a tolerance)

'''

def GetPlaceOnTSRString( global_data, whichmanip, targets ):
    success = 0
    plannerStr = ''
    activeDofs = global_data.armdofs[whichmanip]
    tsrList = []
    loginfo( "Planning to place object on %s" % (targets)  )

    # Get single target name (XXX: extend to multiple targets)
    targetObjects = targets.split()
    if len( targetObjects ) > 1:
        logwarn( "  Selecting first object in list (%s)" % (targetObjects[0]) )
    elif len( targetObjects ) == 0:
        logwarn( "  Must specify at least one target" )
        return success, plannerStr, activeDofs, tsrList

    # Get target transform 
    targetName = targetObjects[0]
    success1, targetPose = GetObjectPose( global_data, targetName )
    T_target = PoseToTransform( targetPose )

    # Get end effector transform 
    manipulators = global_data.robot.GetManipulators()
    T_ee = manipulators[whichmanip].GetEndEffectorTransform()

    if success1 > 0:
        success = 1
        Bw_R = 10 * pi/180
        if 'darpatable' in targetName:
            Bw_T = array( [0.10, 0.10, 0.01] )
        else:
            Bw_T = array( [0.03, 0.03, 0.01] )

        avalues = range( 0, 360, 45 )
        tsrList = GetPlaceOnTSRList( T_target, T_ee, Bw_T, Bw_R, avalues )
        #PrintTSRList( tsrList )
        Bw_T = mat( [-100, 100, -100, 100, -0.05, 100] )
        Bw_R = mat( [-10, 10, -10, 10, -180, 180] ) * pi / 180
        Bw_notilt = hstack( (Bw_T, Bw_R) )
        plannerStr = TSRListToTSRString( global_data, whichmanip, tsrList, Bw_notilt )
    else:
        logwarn( "Objects not in environment!" )

    return success, plannerStr, activeDofs, tsrList



'''
*** Top level function ***

Get manipulation TSR string for manipulating objects such as 
darpastapler, darpaflashlight, darpadrill

'''

def GetManipulationTSRString( global_data, whichmanip, targets, actionName ):
    success = 0
    plannerStr = ''
    activeDofs = global_data.armdofs[whichmanip]
    tsrList = []
    loginfo( "Planning to '%s' object on %s" % ( actionName, targets)  )

    # maximum number of grasps to use for constructing TSR 
    success = 0
    tsrList = []
    plannerStr = ''
    activeDofs = global_data.armdofs[whichmanip]
    success, objPose, tablePose, objName = GetObjectPosesForGrasping(global_data, targets)
    if success:       
        #doIKCheck = False
        doIKCheck = True
        success, graspList, rstr = GetManipGraspList( objName, objPose,
                tablePose, actionName )
            
        if doIKCheck:
            tsrList, plannerStr = GraspitListToIKTSRString( global_data, whichmanip, graspList )
        else:
            maxGrasps = 50
            tsrList = GraspitListToTSRList( graspList, maxGrasps )
            Bw_notilt = []
            plannerStr = TSRListToTSRString( global_data, whichmanip, tsrList, Bw_notilt )

    return success, plannerStr, activeDofs, tsrList, graspList


def MatToStr( vals ):
    return ' '.join('%.3f' % val for val in array( vals.flatten()).flatten() )

'''
Get planner string that will
1. move to pre_grasp_pose 
2. from pre_grasp_pose, it will be possible to move to final_grasp_pose 
   (assumption: final_grasp_pose is translation in the z-direction of
   end-effector.
3. XXX FIXME if preferred backup is not possible, do the best possible 
'''
def GraspitListToIKTSRString( global_data, whichmanip, graspList ):
    robot = global_data.robot

    # save stored active DOF values (note that all grasps use same whichmanip)
    robot.SetActiveDOFs(global_data.armdofs[whichmanip])
    robot.SetActiveManipulator(whichmanip)
    originalActiveDOFValues = robot.GetActiveDOFValues()

    tsrList = []
    targConfigList = []
    plannerStr = ""
    for i in range( len( graspList ) ):
        grasp = graspList[i]

        T_pre = TSRUtil.G2E( PoseToTransform( graspList[i].pre_grasp_pose ) )
        T_fin = TSRUtil.G2E( PoseToTransform( graspList[i].final_grasp_pose ) )
        pushdir = T_fin[0:3][:,3].T - T_pre[0:3][:,3].T
        pushval = sqrt( pushdir * pushdir.T )

        # if T_fin is the same as T_pre, then we don't need to IK checks
        if pushval > 0.0001:
            pushdir = pushdir / pushval
        else:
            logwarn( "  T_pre = T_fin for this grasp (pushval = %.3fm)" % pushval )
            pushdir = T_pre[0:3][:,2].T
            pushval = 0.08

        backupsteps = int( ceil( pushval/0.002 ) ) + 10

        # set target transform 
        T_tar = T_pre

        # Get ik solutions and see if you can move forward from there
        manip = robot.GetManipulators()[whichmanip]
        solutions = manip.FindIKSolutions(array(T_tar),orpy.IkFilterOptions.CheckEnvCollisions)
        if solutions is None or len(solutions) == 0: 
            logwarn( '  Grasp %2d. Found %d IK solutions.' % ( i + 1, len(solutions) ) )
            continue 
        else:
            loginfo( '  Grasp %2d. Found %d IK solutions.' % ( i + 1, len(solutions) ) )

        targConfig = []
        for j,ind in enumerate( range(len(solutions)) ):
            initConfig = solutions[ind]
            robot.SetActiveDOFValues( initConfig )
            logdebug( "     %2d. %s" % ( j+1, MatToStr( solutions[ind] ) ) )
            # Note that we set breakoncollision 0, as we are looking only at
            # the kinematic feasibility of the motion 
            cmd = 'LiftArmGeneralIK breakoncollision 0 exec 0 minsteps %d maxsteps %d direction %s' % (backupsteps, backupsteps, Serialize1DMatrix(mat(pushdir)))
            resp = global_data.probs_manip.SendCommand( cmd )
            finalConfig = util.str2num.str2num( resp )
            robot.SetActiveDOFValues(finalConfig)
            #how far did it move?
            T_act = mat(manip.GetEndEffectorTransform())
            Tdiff = linalg.inv(T_tar)*T_act
            dist = sqrt( ( Tdiff[0:3][:,3] ).T * Tdiff[0:3][:,3] ) 

            if 1:
                logdebug( '          final config = %s' % MatToStr( finalConfig) )
                logdebug( '          IK solution %d: move forward %fm (goal:%fm)' % ( j+1,dist,pushval) )

            #pdb.set_trace()
            if dist >= pushval:
                targConfig = mat( initConfig )
                targConfigList.append( copy.deepcopy( targConfig ) )
                Bw_T = 0.001
                Bw_R = 0.1 * pi / 180
                T0_g = PoseToTransform( grasp.pre_grasp_pose )
                tsr = TSRUtil.GraspitToTSRSimple( T0_g, Bw_T, Bw_R )
                tsrList.append( copy.deepcopy( tsr ) )
                break

    # restore stored active DOF values 
    robot.SetActiveDOFValues(originalActiveDOFValues)

    nGoals = len( targConfigList )
    if nGoals > 0:
        loginfo( 'Constructing TSR string with %d jointgoals' % nGoals )
        nGoalJoints = array( targConfigList[0] ).size * nGoals
        #nGoals = min( 1, nGoals )
        plannerStr = 'jointgoals %d' % nGoalJoints
        for i in range( nGoals ):
            plannerStr = '%s %s' % (plannerStr, Serialize1DMatrix( targConfigList[i] ))
    else:
        logwarn( 'No ik solutions could be backed up far enough, terminating.' )

    return tsrList, plannerStr

# vim: set shiftwidth=4 tabstop=4
