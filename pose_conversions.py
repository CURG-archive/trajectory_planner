import roslib; roslib.load_manifest( "trajectory_planner" )
import geometry_msgs
from tf import transformations as tr
import tf_conversions.posemath as pm
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import opencv
import math

def PoseToTransform( pose ):
    """ 
    Compute 4x4 transform from geometry_msgs::Pose 
    G is a 4x4 matrix which is [R t; 0 0 0 1]
    
    """
    q = pose.orientation
    t = pose.position

    G = tr.quaternion_matrix( array( [ q.x, q.y, q.z, q.w ] ) )
    G[0:3,3] = array( [ t.x, t.y, t.z ] )
    
    return G


def ToPose( t, q ):
    """ 
    Convert t and q to Pose    
    """ 

    orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] ) 
    position = geometry_msgs.msg.Point( t[0], t[1], t[2] ) 
    return geometry_msgs.msg.Pose( position, orientation )




def TransformToPose( G ):
    """ 
    Compute geometry_msgs::Pose from 4x4 transform 
    G is a 4x4 matrix which is [R t; 0 0 0 1]
    
    """ 
    t = array( G )[0:3,3]
    q = tr.quaternion_from_matrix( G )
    orientation = geometry_msgs.msg.Quaternion( q[0], q[1], q[2], q[3] ) 
    position = geometry_msgs.msg.Point( t[0], t[1], t[2] ) 
    pose = geometry_msgs.msg.Pose( position, orientation )
    return pose



def AxesToMatrix( xaxis, yaxis, zaxis, t = array([0,0,0]) ):
    """Compute 4x4 array from x,y,z axis and translation
    
    """ 

    G = eye( 4 )
    G[0:3,0] = xaxis 
    G[0:3,1] = yaxis 
    G[0:3,2] = zaxis 
    G[0:3,3] = t
    return G

def MatrixToAxes( G ):
    """ Return the x,y, and z-axis of the transform
    
    """
    G = array( G )
    xaxis = G[0:3,0]
    yaxis = G[0:3,1]
    zaxis = G[0:3,2]
    t = G[0:3,3]
    return xaxis, yaxis, zaxis, t




def PoseToStr( pose ):
    """ Convert 4x4 transform into a string
    
    """
    t = pose.position
    q = pose.orientation
    fstr = "[ [%.2f %.2f %.2f]  [%.2f %.2f %.2f %.2f] ]" % ( t.x, t.y, t.z, q.x, q.y, q.z, q.w )
    return fstr



def TranToStr( G ):
    '''  Convert 4x4 transform into a string
    
    ''' 

    pose = TransformToPose( G )
    t = pose.position
    q = pose.orientation
    fstr = "[ [%.2f %.2f %.2f]  [%.2f %.2f %.2f %.2f] ]" % ( t.x, t.y, t.z, 
            q.x, q.y, q.z, q.w )
    return fstr



def Rt2G( R, t = mat([0,0,0]) ):
    ''' Compute 4x4 matrix from 3x3 and 3x1
    
    '''

    Rt = hstack( ( R, mat(t).reshape( 3, 1 ) ) )
    return vstack( ( Rt, array( [0, 0, 0, 1] ).reshape( 1, 4 ) ) )


def G2Rt( G ):
    ''' Get 3x3 matrix and 3x1 from 4x4
    
    '''
    R = G[0:3,0:3]
    t = G[0:3,3]
    return R,t


def G2qt( G ):
    """
    Get 3x3 matrix and 3x1 from 4x4 
    """

    q = tr.quaternion_from_matrix( G )
    t = G[0:3,3]
    return q,t


def TransformToScrew( T ):
    """4x4 homogeneous transform to 6x1 plucker line coordinates of screw
    transform
    See http://en.wikipedia.org/wiki/Screw_axis#Screw_axis_of_a_spatial_displacement
    """
    rvec = zeros([3,1])
    d = T[:3,3]
    phi, S, point = pm.transformations.rotation_from_matrix(T)
    b = math.tan(phi/2)*S
    C = (cross(b,d) - cross(b,cross(b,d)))/(2*dot(b,b))
    return vstack([S,cross(C,S)]).flatten()


def PluckerIntersection(P1, P2):
    """Find the point of intersection of two lines parametrized with plucker coordinates.  See http://en.wikipedia.org/wiki/Pl%C3%BCcker_coordinates#Line-line_meet
    """
    d1 = P1[0:3]
    m1 = P1[3:6]
    d2 = P2[0:3]
    m2 = P2[3:6]
    
    return vstack([array([cross(m2,m1)]).transpose(), array([dot(d1,m2)])])
    
    
def TransformIntersection( T_set ): 
    """Given a set of transforms that are rotations around a point,
    estimate that point
    See http://en.wikipedia.org/wiki/Line-line_intersection
    """
    v_set = list()
    p_set = list()
    for T in T_set:
        phi, S, point = pm.transformations.rotation_from_matrix(T)
        p_set.append(array([point]).transpose())
        v_set.append(array([S]).transpose())
    m1 = zeros([3,1])
    m2 = zeros([3,3])
    for k in range(len(p_set)):
        a = eye(3) - dot(v_set[k], v_set[k].transpose())
        m1 += dot(a , p_set[k][:3])
        m2 += a
    return dot(linalg.inv(m2),m1)

def PointsToPlucker(p1, p2):
    d = p2 - p1
    m = np.cross(p1,p2)
    return np.hstack([d,m])

