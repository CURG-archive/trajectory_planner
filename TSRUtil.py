# vim: set shiftwidth=4 tabstop=4
#
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

from numpy import *
import pdb
import sys
from TSRPose import *

'''
Utility to create TSRs for the REARM project
The coordinate frames are 
  0 : Origin 
  w : TSR frame. This can be varied for different objects depending on how the
      bound is specified. The bounds are specified in this coordinate frame. 
  e : End effector coordinate frame 
  p : Palm coordinate frame: This is just an translational offset from
      end-effector coordinate frame. 
  g : The graspit coordinate frame. Graspit returns grasps in this frame.

  Note: T0_e = T0_w * Tw_e 
'''

''' palm pose in graspit coordinates 
'''
__Tg_p = mat( [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] ) 
__Tp_g = linalg.inv( __Tg_p )

''' end-effector pose in palm coordinates  
'''
__Tp_e = mat( [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.135], [0, 0, 0, 1]] )
__Te_p = linalg.inv( __Tp_e )

''' graspit coordinates in end-effector pose 
'''
__Tg_e = __Tg_p * __Tp_e
__Te_g = linalg.inv( __Tg_e )


''' 
Convert to TSR transforms 
'''
def ToTSR( T0_e, T0_w, Bw ):
    Tw_e = linalg.inv( T0_w ) * T0_e          # T0_w * Tw_e = T0_e
    return T0_w,Tw_e,Bw

'''
Same as ToTSR(), except returns TSRPose
'''
def ToTSRPose( T0_e, T0_w, Bw ):
    tsrpose = TSRPose()
    tsrpose.T0_w = T0_w
    tsrpose.Tw_e = linalg.inv( T0_w ) * T0_e
    tsrpose.Bw = Bw
    return tsrpose

'''
'''
def ToBw( Bw_T, Bw_R ):
    Bw = mat( array( ( ToBound( Bw_T ), ToBound( Bw_R ) ) ).flatten() )
    return Bw

'''
'''
def ToBound( B ):
    B = array( B ).flatten()
    if B.size == 1:
        B = array( [-1, 1, -1, 1, -1, 1] ) * fabs( B[0] )
    elif B.size == 3: 
        B = transpose( array( ( -fabs( B ), fabs( B ) ) ) ).flatten()
    elif B.size == 6: 
        B = B.reshape( 6, 1 )
    else:
        B = array( [-1, 1, -1, 1, -1, 1] ) * 0.005
    return B

'''
Functions to transform between Graspit, Palm, and End-Effector coordinates
'''
def E2G( T0_e ):
    return T0_e * __Te_g

def G2E( T0_g ):
    return T0_g * __Tg_e

def E2P( T0_e ):
    return T0_e * __Te_p

def P2E( T0_p ):
    return T0_p * __Tp_e

def G2P( T0_g ):
    return T0_g * __Tg_p

def P2G( T0_p ):
    return T0_p * __Tp_g

'''
Specify the grasp and if applicable, the TSR frame, and associated bounds 
  T0_g: The graspit pose wrt to Origin (pose returned from graspit)
  T0_w: TSR frame (if not necessary use *ToTSRSimple() instead 
  Note that Bw_T can either be a 1x1 or 1x6
'''
def GraspitToTSR( T0_g, T0_w, Bw_T, Bw_R ):
    Bw = ToBw( Bw_T, Bw_R )
    T0_e = T0_g * __Tg_e 
    return ToTSRPose( T0_e, T0_w, Bw )

'''
Specify the grasp and if applicable, the TSR frame, and associated bounds 
  T0_g: The graspit pose wrt to Origin (pose returned from graspit)
  Note: TSR frame (w) is set to palm coordinates 
'''
def GraspitToTSRSimple( T0_g, Bw_T, Bw_R ):
    T0_w = T0_g * __Tg_p
    return GraspitToTSR( T0_g, T0_w, Bw_T, Bw_R )
    


'''
Specify the grasp and if applicable, the TSR frame, and associated bounds 
  T0_p: The palm pose wrt to Origin (pose returned from graspit)
  T0_w: TSR frame (if not necessary use *ToTSRSimple() instead 
  Note that Bw_T can either be a 1x1 or 1x6
'''
def PalmToTSR( T0_p, T0_w, Bw_T, Bw_R ):
    Bw = ToBw( Bw_T, Bw_R )
    T0_e = T0_p * __Tp_e 
    return ToTSRPose( T0_e, T0_w, Bw ) 

'''
Specify the grasp and if applicable, the TSR frame, and associated bounds 
  T0_p: The graspit pose wrt to Origin (pose returned from graspit)
        TSR frame is set to palm coordinates 
'''
def PalmToTSRSimple( T0_p, Bw_T, Bw_R ):
    T0_w = T0_p * __Tg_p
    return PalmToTSR( T0_p, T0_w, Bw_T, Bw_R )


# vim: set shiftwidth=4 tabstop=4
