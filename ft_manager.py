import roslib
roslib.load_manifest("staubliTX60")
import rospy
import geometry_msgs.msg
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg


class FTWatcher(object):
    """@brief - Class to watch the force torque sensor for a large force in a particular direction
    and then run a callback function
    """
    def __init__(self, activated_callback, delay_num = 5, threshold = -14):
        """@brief - Constructor for FTWatcher

           @param activated_callback - Function to run when the threshold in the Z direction is exceeded.
           @param delay_num - Number of consecutive samples above threshold before callback function is activated.
           @param threshold - Threshold above which callback is run.
        """
        self.delay_num = delay_num
        self.hits = delay_num
        self.threshold = threshold
        self.activated_callback = activated_callback
        self.ft_sub = []

    def ft_threshold_func(self, wrench_msg):
        """@brief - Callback function that activates a user specified callback when a threshold in the Z
        direction forces is crossed.

           @param wrench_msg - Received message.
        """
        if wrench_msg.force.z < self.threshold:
            self.hits -= 1
        else:
            self.hits = self.delay_num
        if self.hits == 0:
            print 'ahh'
            self.activated_callback()
            if not self.ft_sub == []:
                sub = self.ft_sub
                self.ft_sub = []
                sub.unregister()
                del sub
        return

    def set_FT_callback(self):      
        """@brief - Register the subscriber for the force-torque sensor.
        """
        self.ft_sub = rospy.Subscriber("/ForceTorque/Readings", geometry_msgs.msg.Wrench, self.ft_threshold_func)

    def unset_FT_callback(self):
        """@brief - Unregister subscriber.
        """
        if self.ft_sub:
            sub = self.ft_sub
            sub.unregister()
            self.ft_sub = []
            del sub
            return True
        return False
                        

def tare_FT(blocking = True, nonblocking_timeout = 0):
    """@brief - Tare the force torque sensor
       @param blocking - whether to use the blocking service or the nonblocking topic to tare the force torque sensor
       @param nonblocking_timeout - the timeout to use if calling the non blocking timoeout
    """    
    if blocking:
        try:
            tare = rospy.ServiceProxy('/ForceTorque/TareService', EmptySrv)
            resp1 = tare()
            return resp1
        except:
            rospy.logwarn('Service Call to tare service failed')
            return 0
    else:
        if tare_FT.ft_tare_pub == []:
            tare_FT.ft_tare_pub = rospy.Publisher("/ForceTorque/Tare", EmptyMsg)
        tare_FT.ft_tare_pub.publish()
        sleep(nonblocking_timeout)
    return 1

tare_FT.ft_tare_pub = []





