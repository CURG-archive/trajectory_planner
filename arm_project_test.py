##!/usr/bin/env python
#import roslib; roslib.load_manifest('trajectory_planner')
#import rospy
#import threading                                                                                      

#from trajectory_planner import *
#from HaoUtil import *

#key_hole_found = False

#def main():
#    global global_data
#    global_data, ftm, tm = start()
#    rospy.loginfo('initialized')
#    move_forward_to_contact(global_data, 0.05)

#def ip_thread():
#    from IPython.Shell import IPShellEmbed
#    ipshell = IPShellEmbed(argv='',banner = 'OpenRAVE Dropping into IPython, variables:' +
#                           'env, robot',exit_msg = 'Leaving Interpreter and ' +
#                           'closing program.')
#    ipshell(local_ns=locals())

#def GrabKey():
#    MoveHandSrv(1,array([1.6,1.6,1.65,1.57]))
#    raw_input('Press enter to try with pelican case collisions turned off ...')
#    MoveHandSrv(1,array([1.8,1.8,1.65,1.57]))
#    pdb.set_trace()

#def InsertKey():
#    '''
#    move left and right
#    move up and down
#    push
#    '''
#    vdist = 0.002
#    hdist = 0.002
#    for i in range(5):

#        #pdb.set_trace()
#        #hit to the surface again
#        #GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.001, 50, False)
#        #move down
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([0,1,0]), array([0,0,-1]), 'PALM', vdist, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.02, 200, False)
#            hole_localized = True
#            break
#        
#        #move to the right
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([1,0,0]), array([0,0,-1]), 'PALM', hdist, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.02, 200, False)
#            hole_localized = True
#            break
#        
#        #hit to the surface again
#        #GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.001, 50, False)        
#        #move up
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([0,-1,0]), array([0,0,-1]), 'PALM', vdist, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.02, 200, False)
#            hole_localized = True
#            break
#        
#        #move to the left
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([-1,0,0]), array([0,0,-1]), 'PALM', hdist, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,-1]),'PALM', 0.02, 200, False)
#            hole_localized = True

#            
#def LocalizeHole( vlevel, hlevel ):
#    #move up and left
#    MoveHand(global_data, array([0,-1,0]), 'PALM', vlevel/2)
#    #we do not want to move to the left a lot
#    MoveHand(global_data, array([-1,0,0]), 'PALM', hlevel/4)
#    
#    #hit to the surface of the key hole
#    GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.1, 30)

#    #scan using move until miss primitive
#    hole_localized = False
#    num_steps = 3
#    for i in range(num_steps):

#        #move down
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([0,1,0]), array([0,0,-1]), 'PALM', vlevel, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.01, 100, False)
#            hole_localized = True
#            break
#        
#        #move to the right
#        if not key_hole_found:
#            MoveHand(global_data, array([0,0,-1]), 'PALM', 0.01)
#        MoveHand(global_data, array([1,0,0]), 'PALM', hlevel/num_steps)
#        GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.015, 30)

#        ## distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([1,0,0]), array([0,0,-1]), 'PALM', hlevel/num_steps, 50, 20, False)
#        ## if isMissed:
#        ##     GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.01, 100, False)
#        ##     hole_localized = True
#        ##     break
#        
#        #hit to the surface again
#        #GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.001, 50, False)        
#        #move up
#        distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([0,-1,0]), array([0,0,-1]), 'PALM', vlevel, 50, 20, False)
#        if isMissed:
#            GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.01, 100, False)
#            hole_localized = True
#            break
#        
#        #move to the right
#        if not key_hole_found:
#            MoveHand(global_data, array([0,0,-1]), 'PALM', 0.01)
#        MoveHand(global_data, array([1,0,0]), 'PALM', hlevel/num_steps)
#        GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.015, 30)

#        ## distance, isMissed, isHit = GuardedMoveUntilMiss(global_data, array([1,0,0]), array([0,0,-1]), 'PALM', hlevel/num_steps, 50, 20, False)
#        ## if isMissed:
#        ##     GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.01, 100, False)
#        ##     hole_localized = True

#    return hole_localized

#def LocalizeHandle1():
#    '''
#    first version, move the hand to the left end of the handle
#    '''

#    #move forward to hit the door
#    GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.15)
#    rospy.loginfo('door hit')

#    #move back 
#    MoveHand(global_data, array([0,0,-1]),'PALM', 0.045)

#    #move down to hit the handle
#    GuardedMoveUntilHit(global_data, array([0,1,0]),'PALM', 0.5)
#    rospy.loginfo('handle hit')

#    #move up 2 cm
#    MoveHand(global_data, array([0,-1,0]),'PALM', 0.02)
#    #move to the left
#    MoveHand(global_data, array([-1,0,0]), 'PALM', 0.1)
#    #move down to the handle range
#    MoveHand(global_data, array([0,1,0]), 'PALM', 0.03)
#    #move forward a little so that the knuckle hits the handle side
#    MoveHand(global_data, array([0,0,1]), 'PALM', 0.03)
#    #move to the right to hit the handle
#    GuardedMoveUntilHit(global_data, array([1,0,0]),'PALM', 0.2)

#    #move to the left again
#    MoveHand(global_data, array([-1,0,0]), 'PALM', 0.01)
#    #move back
#    MoveHand(global_data, array([0,0,-1]), 'PALM', 0.06)
#    #move to the right
#    MoveHand(global_data, array([1,0,0]), 'PALM', 0.13)

#def LocalizeHandle2():
#    '''
#    second version, move the hand to the right end of the handle
#    '''
#    
#    #move forward to hit the door
#    GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.15)
#    rospy.loginfo('door hit')

#    #move back 
#    MoveHand(global_data, array([0,0,-1]),'PALM', 0.045)

#    #move down to hit the handle
#    GuardedMoveUntilHit(global_data, array([0,1,0]),'PALM', 0.5)
#    rospy.loginfo('handle hit')

#    #move up
#    MoveHand(global_data, array([0,-1,0]),'PALM', 0.04)
#    #move to the right
#    MoveHand(global_data, array([1,0,0]),'PALM', 0.12)
#    #move forward
#    MoveHand(global_data, array([0,0,1]),'PALM', 0.035)

#    #move to the left
#    GuardedMoveUntilHit(global_data, array([-1,0,0]),'PALM', 0.1)
#    #move to the right to get away from the contact
#    MoveHand(global_data, array([1,0,0]), 'PALM', 0.01)
#    #move down
#    GuardedMoveUntilHit(global_data, array([0,1,0]),'PALM', 0.1)

#    #retract
#    MoveHand(global_data, array([0,0,-1]),'PALM', 0.08)
#    #-x: -0.02201925
#    MoveHand(global_data, array([-1,0,0]),'PALM', 0.032)
#    #-y: 0.01447249
#    MoveHand(global_data, array([0,1,0]),'PALM', 0.038)



#if __name__ == '__main__':
#    NODE_NAME='test'
#    #MoveHandSrv(1,[3.1,3.1,3.1,0])
#    #rospy.init_node(NODE_NAME)
#    #ipython_thread = threading.Thread(None, ip_thread)
#    #ipython_thread.start()
#    #rospy.spin()
#    #global global_data
#    global_data, ftm, tm = start()
#    rospy.loginfo('initialized')
#    #MoveHand(global_data, array([0,0,-1]),'PALM', 0.01)
#    pdb.set_trace()
#    ## return
#    ## ReOrient2(global_data,20.*pi/180.)
#    ## ReOrient2(global_data,20.*pi/180.)
#    
#    ## #key insertion
#    ## #GrabKey()
#    ## pdb.set_trace()
#    ## LocalizeHandle2()
#    ## LocalizeHole(0.01, 0.01)
#    ## LocalizeHole(0.002, 0.005)
#    ## InsertKey()
#    ## while not rospy.is_shutdown():
#    ##     rospy.sleep(1.0)


#def NaiveScan():
#    #let's begin the scanning
#    dx = 0.0015
#    dy = 0.0015
#    done = False
#    for x_steps in range(4):
#        if done == True:
#            break
#        #scan horizontal lines
#        MoveHand(global_data, array([1,0,0]), 'PALM', dx)
#        MoveHand(global_data, array([0,-1,0]), 'PALM', 2*dy)
#        for y_steps in range(4):
#            #scan vertical lines
#            MoveHand(global_data, array([0,1,0]), 'PALM', dy)
#            #hit the hole
#            dist, isHit = GuardedMoveUntilHit(global_data, array([0,0,1]),'PALM', 0.07, 150)
#            rospy.loginfo('distance moved is: %s', dist)
#            if dist > 0.04:
#                rospy.logerr('I believe that I inserted the key')
#                done = True
#                break
#            #move back a little bit
#            MoveHand(global_data, array([0,0,-1]), 'PALM', 0.03)

#        #move up the the original height
#        MoveHand(global_data, array([0,-1,0]), 'PALM', 2*dy)



#def new_test( grasp_tran = [], visualize_openrave = True, object_name = 'flask' ):
#    """ Move the hand to a pregrasp position.  If no grasp is given, moves the hand 5 cm above the object with it's palm pointed down.
#    """

#    global_data = SetupStaubliEnv(visualize_openrave)

#    object_name = file_name_dict['object_name']

#    if not grasp_tran:
#        grasp_tran = eye(4)
#        grasp_tran[1,1] = -1
#        grasp_tran[2,2] = -1
#        grasp_tran[2,3] = get_object_height(global_data.or_env, 'object', file_name_dict[object_name])

#    pregrasp_object(global_data, file_name_dict[object_name], grasp_tran, run_it = True)

#    return


#    
#    
