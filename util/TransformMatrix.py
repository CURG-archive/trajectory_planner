# -*- coding: utf-8 -*-

# Copyright 2009-2010 Carnegie Mellon University and Intel Corporation
# Author: Dmitry Berenson
#

'''Functions for defining, extracting data from, and serializing transform matrices'''
from numpy import *

def MakeTransform(rot,trans):
    if size(rot,0) == 9 and size(rot,1) == 1:
        tm = rot.reshape(3,3).T
    elif size(rot,0) == 3 and size(rot,1) == 3:
        tm = rot
    else:
        print('rotation improperly specified');

    if size(trans,0) == 3 and size(trans,1) == 1:
        tm = bmat('tm trans')
    elif size(trans,0) == 1 and size(trans,1) == 3:
        tempTrans = trans.T
        tm = bmat('tm tempTrans')
    else:
        print('translation improperly specified');
    
    lastrow = mat([0,0,0,1])
    return bmat('tm; lastrow')

def GetRot(tm):
    return mat(tm[0:3][:,0:3].T.reshape(1,9));

def GetTrans(tm):
    return mat(tm[0:3][:,3].T);

def SerializeTransform(tm):
    rot = GetRot(tm)
    trans = GetTrans(tm)
    rottrans = bmat('rot trans')
    return Serialize1DMatrix(rottrans)

def Serialize1DMatrix(arr):
    return '%s'%(' '.join(' %.5f'%(arr[0,f]) for f in range(0,size(arr))))
