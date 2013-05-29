from numpy import *

class TSRPose():
    def __init__(self, T0_w_in = mat(eye(4)), Tw_e_in = mat(eye(4)), Bw_in = mat(mat(zeros([1,12])))):
        self.T0_w = T0_w_in
        self.Tw_e = Tw_e_in
        self.Bw = Bw_in

    def T( self ):
        return self.T0_w * self.Tw_e

