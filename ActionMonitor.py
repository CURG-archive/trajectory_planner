PKG= 'owd'
import roslib; roslib.load_manifest(PKG)
roslib.load_manifest('staubli_tx60')
import rospy
from rospy import loginfo
import numpy as np
import pr_msgs.msg
import std_msgs.msg
import pr_msgs.srv
import actionlib
import geometry_msgs.msg
import OWDUtil
import pdb
from pr_msgs.srv import SetSpeed, MoveHand

global_channel_list = dict()
global_channel_list['HandState'] = '/bhd/handstate'
global_channel_list["Tactile"] = '/bhd/tactile'

class DataModel(object):
    def name_(self):
        return None
    """Class to encapsulate a name of a model with a function that instantiates it"""
    def __call__(self, original_data, internal_params = []):
        return None
    def get_name(self):
        return self.name_()
    def warn(self):
        return self.name_()
    def update_params(self, params):
        if params is None or len(params) == 0:
            return True
        self.num_internal_params = params[0]
        if len(params) >= self.num_internal_params:
            self.internal_params = params[1:self.num_internal_params+1]
            #if all params are consumed, return
            if len(params) == self.num_internal_params+1:
                return True
            #if this model contains a model, send it the rest of the params
            if hasattr(self,"model"):
                return self.model.update_params(params[self.num_internal_params:])
            #if there are excess parameters, eat them
            return True
        #if there are not enough parameters fed to this model, complain
        return False
    def get_channel(self):pass
    def close(self): pass

class RecursiveModel(DataModel):
    """Base class describing interface for the recursive model class

    The design of the hierarchy of classes is meant to encapsulate data stream processing for
    messages.

    Each member must provide at least these methods and members
    """
    def __init__(self, model, internal_params = []):
        self.model = model
        self.value = []
        self.internal_params = []
        self.update_params(internal_params)
        
    def  __call__(self, original_data,  params = []):pass
    """This function should follow the pattern
    new_processed_data = model(original_data, params[self.consumed_params:])
    return this_model(original_model, new_processed_model,
                        params[self.consumed_params], self.internal_params)
    """
    
    def get_name(self):
        return "%s/%s"%(self.model.get_name(), self.name_())

    def warn(self):
        return "%s/%s"%(self.model.warn(), self.get_name())

    def close(self):
        self.model.close()

    def get_channel(self):
        return self.model.get_channel()

    def __del__(self):
        self.close()
    


class SensorPredicateManager(RecursiveModel):
    """Effectively a decorator that adds some syntactic sugar to models
    providing an interface to update or retrieve the parameters for the model from
    the ros parameter server and publishing warning messages to any external
    watchers when the predicate being handled returns false
    """    
    def __init__(self, model, internal_params = [], warning_channel_name = None):
        """initialize from model object"""
        RecursiveModel.__init__(self, model, internal_params)
        try:
            self.update_model_service =  rospy.Subscriber(self.model.get_name() + "/UpdateParameters",std_msgs.msg.Empty, self.update_model_params_callback)
            if warning_channel_name == None:
                warning_channel_name = self.model.get_name() + "/Warning"
            self.warning_pub = rospy.Publisher(warning_channel_name, std_msgs.msg.String)
            self.base_name=""
            
        #If initialization fails, it is important to unregister the service
        except Exception as e:
            import pdb
            pdb.post_mortem()
            self.update_model_service.shutdown()
            self.warning_pub.unregister()
                
        
    def __call__(self, original_data,  params = []):
        """
        call the given predicate, determine any of its return values are true, publish a warning
        and return true

        """
        try:
            self.value = self.model(original_data,  params)
        except TypeError:
            import pdb
            pdb.post_mortem()
        if not self.value:
            self.warning_pub.publish(
                         std_msgs.msg.String(self.model.warn()))
        return (self.value > 0)
        
        
    def update_model_params_callback(self, msg = None):
        """
        callback function to update the current set of model parameters
        """
        try:
            self.update_params(rospy.get_param("/ModelParams/%s"%(self.model.get_name())))
        except KeyError:
            print "Failed to find params for model %s" % (self.model.get_name())
        #Some models don't have parameters, it is ok for them to fail here
        return True

    def close(self):
        self.update_model_service.unregister()
        self.warning_pub.unregister()
        self.model.close()
        #unregister the model to make the GCs life easier

    def warn(self):
        self.model.warn()
        
    def __del__(self):
        self.close()
        


        
class PolynomialFitErrorDataModel(RecursiveModel):
    """ internal_params = [num_params = order + 1, cOrder, cOrder-1 ... c0]
        data = [norm(expected data - polynomial_fit to data)
    """
    
    def name_(self):
        return "polynomial"
    
    def __init__(self, model, internal_params= [0]):        
        RecursiveModel.__init__(self, model, internal_params)


    def __call__(self, original_data, params = []):
        vals = self.model(original_data, params)
        expected_data = vals[0]
        dependent_data = vals[1]
        order = self.num_internal_params -1
        expected_y = np.array([np.dot(expected_data**(order-i),self.internal_params[order - i + 1]) for i in range(order)])
        #dot product may return a scalar - by convention we always return np arrays
        self.value =  np.array(np.linalg.norm(dependent_data - expected_y)) 
        return self.value

class AffineModel(RecursiveModel):
    """Multiply the output of a datasource by an affine transform

    """

    def name_(self):
        return "affine"

    def __init__(self, model, internal_params = [0]):
        """@params - [size_affine_matrix + 2, shape[0], shape[1], row_major_matrix_entries...]
        """
        self.shape = []
        self.affine = []
        RecursiveModel.__init__(self, model, internal_params)

    def update_params(self, params):
        RecursiveModel.update_params(self, params)
        self.shape = self.internal_params[0:2]
        self.affine = np.array(self.internal_params[2:]).reshape(self.shape)


    def __call__(self, original_data, params):
        vals = self.model(original_data, params)
        return np.array(np.dot(self.affine, vals))


class TimeDelayModel(RecursiveModel):
    """Wait a specified amount of time before allowing a false

    """
    def name_(self):
        return "timedelay"

    def __init__(self, model, internal_params = [1,0]):
        RecursiveModel.__init__(self, model, internal_params)
        self.start_time = rospy.rostime.time.time()
        
    def __call__(self, original_data, params):
        now = rospy.rostime.time.time()
        duration = self.internal_params[0]

        if now - self.start_time  > duration:
            return self.model(original_data, params)
        else:
            return True



class SampleDelayModel(RecursiveModel):
    """Wait a specified number of samples before switching output

    """
    def name_(self):
        return "sampledelay"

    def __init__(self, model, internal_params = [1,0]):                
        RecursiveModel.__init__(self, model, internal_params)
        self.samples_left_to_switch = self.internal_params[0]
        self.state = True
        
    def __call__(self, original_data, params):
        value =  self.model(original_data, params)
        if value == self.state:
            self.samples_left_to_switch = self.internal_params[0]
        else:
            self.samples_left_to_switch -= 1
        if self.samples_left_to_switch <= 0:
            self.state = not self.state
            rospy.logwarn("Switched")
            self.samples_left_to_switch = self.internal_params[0]
        return self.state



class NormModel(RecursiveModel):
    def name_(self):
        return "norm"
    def __init__(self, model, internal_params = []):
        RecursiveModel.__init__(self, model, internal_params)
        self.num_internal_params = 0
        
    def __call__(self, original_data, params):
        vals = self.model(original_data, params)
        return np.array(np.linalg.norm(vals))

class ThresholdDataModel(RecursiveModel):
    """
    Boolean test for data within a threshold
    """
    def name_(self):
        return "threshold"
    def __init__(self, model, internal_params = [0]):
        RecursiveModel.__init__(self, model, internal_params)
        
    def __call__(self, original_data, params):
        new_processed_data = self.model(original_data, params = [])
        threshold = np.array(self.internal_params)
        try:
            if new_processed_data.size != threshold.size:
                raise TypeError()
        except:
            pdb.set_trace()
        self.value = new_processed_data.flatten() - threshold.flatten()
        return  (self.value < 0).all()


class AbsThresholdDataModel(ThresholdDataModel):
    """Compare the absolute value of a datasource to a threshold.
    """
    
    def name_(self):
        return "absthreshold"

    def __call__(self, original_data, params):
        new_processed_data = self.model(original_data, params = [])
        threshold = np.array(self.internal_params)
        if new_processed_data.size != threshold.size:
            print "Wrong size data"
            print new_processed_data.size, threshold.size
            raise TypeError()
        self.value = new_processed_data.flatten() - threshold.flatten()
        return any(abs(self.value) > 0)

class BooleanEquivalenceModel(RecursiveModel):
    """
    A test for boolean equivalence between data[1] and data[2]
    """
    
    def name_(self):
        return "allequal"
    def __call__(self, original_data, params):
        data = self.model(original_data, params)
        return all(data[0] == data[1])
            



class DisjointSetsModel(RecursiveModel):
    """
    A test for disjointness between data[1] and data[2]
    """
    
    def name_(self):
        return "disjointsets"
    def __call__(self, original_data, params = []):
        data = self.model(original_data, params = [])
        return not any( [d1 == d2 for d1 in data[0] for d2 in data[1]] )





class OverlappingSetsModel(RecursiveModel):
    """
    Test for overlap between data[1] and data[2]
    """
    
    def name_(self):
        return "overlappingsets"
    def __call__(self, original_data, params = []):
        data = self.model(original_data, params)
        return any( [d1 == d2 for d1 in data[0] for d2 in data[1]])

    
class SensorModel(DataModel):
    """Base class for sensor models
    Models encapsulate the base channel name, the model type, the data generator for the model
    and the base model function
    """
    def name_(self):
        pass
    def __init__(self, channel, internal_params = []):
        self.channel = channel
        self.internal_params = []
        self.update_params(internal_params)
    def __call__(self, msg, params = []):pass
    """Descendents of this class should specialize this function"""
    

    def warn(self):
        return self.get_name()

    def get_name(self):
        return "%s" % (self.name_())

    def get_channel(self):
        return self.channel



class FingerModel(SensorModel):
    """Base class for sensors attached to a finger.  All derived classes
    expect the finger number to be the last parameter passed in

    """
    def name_(self):
        pass
    def __init__(self, channel = None, internal_params = None):        
        self.update_params(internal_params)        
        #augment the action base name with the fingernumber       
        SensorModel.__init__(self, channel, internal_params)
    def get_name(self):
        return "finger_%d/%s"%(self.finger_num, self.name_())
        
    def update_params(self, params):
        SensorModel.update_params(self, params)
        self.finger_num = self.internal_params[-1]

    
    
           

class StrainFingerModel(FingerModel):
    def __init__(self, internal_params):
        channel = global_channel_list["HandState"]
        FingerModel.__init__(self, channel, internal_params = internal_params)        
    def __call__(self, msg, params):
        return np.vstack([np.array(msg.strain), np.array(msg.positions[:3])]).transpose()[self.finger_num,:]
    def name_(self):
        return "strain"




class PositionFingerModel(FingerModel):
    
    def __init__(self, internal_params):
        channel = global_channel_list["HandState"]
        self.target_position = []
        self.internal_params = internal_params
        FingerModel.__init__(self, channel, internal_params)

    def __call__(self, msg, params = []):
        return np.array([self.target_position, msg.positions[self.finger_num]])

    def update_params(self, params):
        FingerModel.update_params(self, params)
        self.target_position = self.internal_params[1]
        
    def name_(self):
        return  "position"

class FingerStateModel(FingerModel):
    
    def __init__(self, internal_params):
        self.target_state = []
        channel = global_channel_list["HandState"]
        FingerModel.__init__(self, channel, internal_params)


    def name_(self):
        return  "state"

    def update_params(self, params):
        FingerModel.update_params(self, params)
        self.target_state = self.internal_params[1:-1]

    def  __call__(self, msg, params = []):
        return [self.target_state, [msg.internal_state[self.finger_num]]]



class TactileFingerModel(FingerModel):
    
    def name_(self):
        return  "tactile"
    def __init__(self, internal_params):
        channel = global_channel_list["Tactile"]
        FingerModel.__init__(self, channel, internal_params)
    def __call__(self, msg, params = []):
        return np.array(eval("msg.finger%d" % (self.finger_num + 1)))





class ActionLibModel(SensorModel):
    def __init__(self, base_action_channel, action_type, internal_params = []):
        self.base_action_channel = base_action_channel
        self.channel = "%s/feedback"%(base_action_channel)
        self.action_type = action_type
        self.action_client = actionlib.SimpleActionClient(self.base_action_channel, self.action_type)
        self.update_params(internal_params)
        self.goal = self.get_goal()

    def get_goal(self):pass            

    def __call__(self, msg, params = []):pass

    def close(self):
        self.action_client.cancel_goal()

    def __del__(self):
        self.close()
    

class ForceTransducerModel(SensorModel):
    """Abstracts force torque sensor data
    """
    def name_(self):
        return "forcetorque"
    def __init__(self, internal_params = np.array([7,0,1,2,3,4,5,6])):
        """@param internal_params - expects [(use_time)+num_ft_channels, channel_ind1, channel_ind1...] - index 0 corresponds to sample time 
        """
        channel ="/right/owd/forcetorque"
        self.indeces = []
        SensorModel.__init__(self, channel, internal_params)
        
    def __call__(self, msg, params = []):
        return np.array([msg.header.stamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.force.y, msg.wrench.force.z])[self.indeces]
    def update_params(self, params):
        SensorModel.update_params(self, params)
        self.indeces = np.array(self.internal_params)
        

class WamStateModel(SensorModel):
    """Abstracts watching for an expected wam state. Produces [expected states , observed state]
    """
    def __init__(self, internal_params):
        """@param internal_params - expects [1, state_enum]
        """
        SensorModel.__init__(self, channel, internal_params)
        self.target_state = internal_params[1]

    def update_params(self, params):
        SensorModel.update_params(self, params)
        self.target_state = internal_params[1,:]

    def  __call__(self, msg, params = []):
        return [self.target_state, msg.state]






def get_topic_type(topic_name):
    """Helper function to produce the topic message type from the topic
    name using ros introspection capabilities
    @param topic_name - valid ros topic to introspect
    @type topic_name - string
    """
    
    published_types = rospy.get_published_topics()

    topic_info = [t for t in published_types if t[0]==topic_name]
    
    
    #an unknown topic name should throw.  Someone upstream should handle this
    if not topic_info:
        raise rospy.topics.ROSException(topic_name)
    topic_type_string = topic_info[0][1]
    package_name = topic_type_string.split('/')[0]
    message_name = topic_type_string.split('/')[1]
    topic_type = eval(package_name+ '.msg.' + message_name)
    return topic_type



class ActionMonitor(object):
    """ Encapsulates the registration and destruction of subscribers to messages for watching
     an action
    """
    class SubscriberCallbackManager(object):
        """Acts as a gatekeeper and custodian for topics - Effectively, this class
        acts a proxy for creating multiple subscribers for a particular topic
        and allows association between all of the subscribers for a particular topic
        """
        def __init__(self, owner):
            """
            @param owner - the ActionMonitor that owns this SubscriberCallbackManager
            """
            self.callback_list = list()
            self.owner = owner
            self.valid = True
            
        def __call__(self, msg):
            """ Callback function for the managed topic

            This function applies the predicates registered with it
            to the newest message from a topic and
            tracks which predicates fail.  It then calls on the ActionModel
            which owns it have an opportunity to respond to the failing predicates
            somehow
            
            @param msg - msg from topic
            
            """
            failing_predicates = list()
            for c in self.callback_list:
                if not c(msg):
                    failing_predicates.append(c)
            
            self.owner.handle_predicate_failures(failing_predicates)
            #Test for timeouts, if they are enabled
            if self.owner.start_time + self.owner.duration < rospy.rostime.time.time():
                self.owner.timeout_state = True
                self.owner.close()
            
        def __len__(self):
            return len(self.callback_list)
            
        def append(self, subscriber_predicate):
            """Add a predicate to be applied to this topic
            """
            self.callback_list.append(subscriber_predicate)

        def remove(self, subscriber_predicate):
            """Remove a predicate from those applied to this topic
            """
            self.callback_list.remove(subscriber_predicate)


    def get_name(self):
        return ""
           
    def __init__(self, update_before_launch = False, timeout = float('inf'), warning_channel_name = None):
        """
        @param update_before_launch - Flag to force the predicates
        created by this ActionModel to update their parameters from the parameter
        server before launching.  Used by models that with dynamic parameters
        for models that are fit using some external script.

        @param timeout - Set a time after which this action will close itself

        @param warning_channel_name - a global warning channel for all predicate managers,
        if one is given.  Otherwise, the predicate managers will pick their own

        @member subscriber_callback_monitor - A dictionary of SubscriberCallbackMonitors
        keyed by the topic name of the monitored topic.

        @member sensor_predicate_list - a list of predicates that correspond to
        the expected behavior of this action.  

        @member start_time - Track the start time of this action for testing timeouts

        @member duration - Expected length of action for timeouts        
        
        """        
        self.subscriber_callback_monitor = dict()
        self.sensor_predicate_list = []
        self.subscriber_dictionary = dict()
        self.build_model()
        self.start_time = rospy.rostime.time.time()
        self.duration = timeout
        self.timeout_state = False
        self.warning_channel_name = warning_channel_name
        for sp in self.sensor_predicate_list:
            sp.base_name = self.get_name()
            if update_before_launch:
                sp.update_model_params_callback()
                
            if not self.subscriber_dictionary.has_key(sp.get_channel()):
                self.subscriber_callback_monitor[sp.get_channel()] = self.SubscriberCallbackManager(self)
                sub = rospy.Subscriber(sp.get_channel(), get_topic_type(sp.get_channel()),
                                       self.subscriber_callback_monitor[sp.get_channel()], queue_size = 50)
                
                self.subscriber_dictionary[sp.get_channel()] = sub
                
            self.subscriber_callback_monitor[sp.get_channel()].append(sp)
        self.action_name = None
     
               
    def handle_predicate_failures(self, failing_predicate_list):
        """Do something here if this action knows what to do in response to a failure
        of a particular type of predicate.  I.E., print a warning,
        call a service to stop the arm, open the hand, or just mark the action completed
        and close it.
        """
        return None

    def close(self):
        """Do some explicit cleanup that attempts to make life easier
        for the garbage collector and reduce load on the rospy infrastructure
        """
        for sub in self.subscriber_dictionary:
            self.subscriber_dictionary[sub].unregister()
        for pred in self.sensor_predicate_list:
            pred.close()
            self.remove_predicate(pred)
            
        for scm_key in self.subscriber_callback_monitor:
            #remove cyclical dependencies to allow the GC to clean up this action
            #more easily
            scm = self.subscriber_callback_monitor[scm_key]
            del scm
            self.valid = False
            return self.valid

    def __del__(self):
        self.close()


    def build_model(self):
        """Define your expected sensor behavior in terms or sensor predicates built
        out of sensor models.
        """
        return None

    def remove_predicate(self, sp):
        """remove a predicate from the subscriber callback list
        Unsubscribes from topic if no more predicates for that topic exist.
        
        @param sp - predicate to remove
        """
        self.subscriber_callback_monitor[sp.get_channel()].remove(sp)
        if len(self.subscriber_callback_monitor[sp.get_channel()]) == 0:
            dead_monitor = self.subscriber_callback_monitor.pop(sp.get_channel())
            self.subscriber_dictionary[sp.get_channel()].unregister()
        self.sensor_predicate_list.remove(sp)

    def num_active_predicates(self):
        """ Track the number of active predicates
        """
        return len(self.sensor_predicate_list)



    def timed_out(self):        
        """Convenience function for higher level code to query the state of the action.
        Base class implementation is not definitive
        """
        return self.timeout_state

    def succeeded(self):
        """Convenience function for higher level code to query the state of the action.
        Base class implementation is not definitive
        """
        return self.completed() and not self.timed_out()

    def completed(self):
        """Convenience function for higher level code to query the state of the action.
        Base class implementation is not definitive
        """
        return self.num_active_predicates() == 0

"""
Set of convenience functions to create sets of models
"""
    
import functools


def bind_to_factory(model_factory, **mykwds):
    """Given a functor, bind the given keywords to the functor.  Useful for
    binding a set of a parameters to the functor that will create
    models that express the appropriate parameters to a higher level function
    that decides which sensor input the model is applied to.

    Nothing about this is specific to models, it is just a wrapper around
    a command that some people might find imposing
    """
    return functools.partial(model_factory, **mykwds)
    


def process_factory_list(factory_list, base_factory):
    """ Given a list of model factory objects, it cascades the output from
    one to another.  Outputs a factory of composed models

    @param base_factory - lowest level factory, usually a sensor_model
    @param factory_list - a set of factories that are meant to be
    composed on the factory before them.  

    Example:
    factory_list = [SampleDelay, PolynomialFitErrorDataModel]
    base_factory = bind_to_factory(PositionFingerModel, internal_params=[1,.5])
    process_factory_list(factory_list, base_factory)
    """
    final_factory = base_factory
    for fac in reversed(factory_list):
        final_factory = functools.partial(fac, model=final_factory())
    return final_factory


class HandActionMonitor(ActionMonitor):
    """Extension of action monitor with some helper functions
    for setting up factories for each finger
    """
    def setup_finger_models(self, finger_sensor_model_factory,
                            finger_sensor_params = [],
                            model_list = [],
                            active_fingers = range(3)):
        """
        Given a set of fingers, set up the given model for each finger in the set
        @param finger_sensor_model - the base sensor model for each finger.  expects a factory for
        classes that inherit from FingerSensorModel

        @param finger_sensor_params - set of parameterse for the finger model
        
        @param model_list - A list of models to be composed for each finger
        @ref process_factory_list

        @param active_fingers - list of finger indices to apply the given models to.
        
        """

        for i in active_fingers:
            param = [1, i]
            if finger_sensor_params:
                param + finger_sensor_params[i]
                param[0] += len(finger_sensor_params)
            param.append(i)
            model = bind_to_factory(finger_sensor_model_factory, internal_params = param)
            
            f = process_factory_list(model_list, model)
            sp = SensorPredicateManager(f())
            self.sensor_predicate_list.append(sp)
        return self.sensor_predicate_list

    def get_finger_num(self, name_str):
        """
        Helper function to return the finger index of a given model from its name
        @param name_str - name of the model
        """
        finger_ind = name_str.find("finger_")        
        if finger_ind > -1:
            finger_num = int(name_str[finger_ind+7])
        else:
            finger_num = -1
        return finger_num
    
    def unregister_finger(self, finger_num):
        """Disable all callbacks respecting finger finger_num
        @param finger_num - the index of the finger to remove
        """
        for sp in self.sensor_predicate_list:
            sp_finger_num = self.get_finger_num(sp.get_name()) 
            if sp_finger_num == finger_num:
                self.remove_predicate(sp)
    


class GuardedHandVelocityMotion(HandActionMonitor):
    """
    Close the hand at a given velocity until a position or contact occurs
    This model requires parameters from a parameter server because the strain gauges
    and tactile sensors on some hands are nosier than others and the
    model may need to be retrained during execution of the test
    """
    def get_name(self):
        return "VelocityMotion"
    
    def __init__(self, desired_positions, movement_velocities):
        """
        @param desired_positions - the goal positions for each finger.  3x1 numpy array
        @param movement_velocities - the velocity to move those fingers which are not in contact
        
        """
        self.desired_positions = desired_positions
        self.movement_velocities = movement_velocities
        MoveHandSrv(1, self.desired_positions)
        SetHandSpeed(self.movement_velocities)
        HandActionMonitor.__init__(self, True)

        
    def build_model(self):
        self.setup_finger_models(StrainFingerModel, model_list = [ThresholdDataModel, PolynomialFitErrorDataModel])
        TactileThresholdModel = bind_to_factory(ThresholdDataModel, internal_params = [24] + np.zeros([24,1]).tolist())
        self.setup_finger_models(TactileFingerModel,  model_list = [TactileThresholdModel])
        PositionThresholdModel = bind_to_factory(AbsThresholdDataModel, internal_params = [1, .01])
        PositionTargetModel = bind_to_factory(PolynomialFitErrorDataModel, internal_params = [2, 0, 1])
        self.setup_finger_models(PositionFingerModel, finger_sensor_params = [[i] for i in self.desired_positions], model_list = [PositionThresholdModel, PositionTargetModel])
        self.setup_finger_models(FingerStateModel, finger_sensor_params = [[[pr_msgs.msg.BHState.state_stalled]] for i in range(3)], model_list = [DisjointSetsModel])
        
        
    def handle_predicate_failures(self, failure_list):
        for f in failure_list:
            finger_ind = self.get_finger_num(f.get_name())
            #remove all predicates watching this finger
            self.movement_velocities[finger_ind] = 0
            self.unregister_finger(finger_ind)
            current_position = GetHandPosition()[finger_ind]
            self.desired_positions[finger_ind] = current_position
            #reissue movement command with this finger at its 
        if len(failure_list) > 0:
            MoveHandSrv(1, self.desired_positions)
                
    def completed(self):
        return self.num_active_predicates() == 0
    





class GuardedFTMotion(ActionMonitor):
    def get_name(self):
        return "FTArm"
        
    def __init__(self, world_direction, end_effector_transform, threshold, update_models = False, time_delay = None, sample_delay = None, timeout = float('inf')):
        """
        @param world_direction - direction in world coordinates to watch for a force.  WARNING - the actual calculation is done
        in end effector coordinates, so if the hand pose changes, the watched direction does also.
        
        @param end_effector_transform - starting end effector transform.  Used to extract the rotation between the world
        and the end effector

        @param threshold - magnitude of force above which a warning is generated and motion is stopped

        @param update_models - unused  FIXME

        @param time_delay - Do not generate warnings up to a specified time - used to allow some leeway for the beginning of jerky
        motion.

        @param sample_delay - Do not generate warnings unless more than this number of samples over the threshold have been seen.
        Used to prevent sensor noise and unmodelled dynamics from stopping motion prematurely

        @param timeout - expected time the motion should take.  Disables monitor after specified amount of time.  Does not stop motion
        

        """
        self.hand_direction = np.dot(np.linalg.inv(end_effector_transform[:3,:3]), world_direction)
        self.threshold = threshold
        self.time_delay = time_delay
        self.sample_delay = sample_delay
        
        ActionMonitor.__init__(self, update_models, timeout = timeout)

    def build_model(self):
        #looking only at force
        base_force = ForceTransducerModel([3] + range(1,4))

        force_opposite_direction = AffineModel(base_force, [5] + [1,3] +  self.hand_direction.tolist())
        force_threshold = ThresholdDataModel(force_opposite_direction, [1, self.threshold])
        top_level_model = force_threshold
        if self.sample_delay != None:
            top_level_model = SampleDelayModel(top_level_model, [1, self.sample_delay])
        if self.time_delay != None:
            top_level_model = TimeDelayModel(top_level_model, [1, self.time_delay])
            
        sp = SensorPredicateManager(top_level_model)
        self.sensor_predicate_list.append(sp)

    def handle_predicate_failures(self, failure_list):
        for f in failure_list:
            self.remove_predicate(f)
            cancel = rospy.ServiceProxy('/right/owd/CancelAllTrajectories', pr_msgs.srv.CancelAllTrajectories)
            try:
                cancel()
            except:
                rospy.logwarn("Trajectories: Failed to clear paused trajectories")
            self.close()


class KnockOffGuard(ActionMonitor):
    def get_name(self):
        return "KnockOffGuard"
    
    def __init__(self, update_models = False, sample_delay = 5, warning_channel_name = None):
        self.__knock_off_detected = False
        self.sample_delay = sample_delay
        ActionMonitor.__init__(self, update_models, warning_channel_name = warning_channel_name)


        
    def build_model(self):
        clanks = AudioDetectorModel([2, global_sound_dict["pipe"],global_sound_dict["hammer"]])
        clanks_predicate = DisjointSetsModel(clanks)
        top_level_model = clanks_predicate
        if self.sample_delay:
            top_level_model = SampleDelayModel(top_level_model, [1, self.sample_delay])

        sp = SensorPredicateManager(top_level_model, warning_channel_name = self.warning_channel_name)                
        
        self.sensor_predicate_list.append(sp)
        

    def handle_predicate_failures(self, failure_list):
            for f in failure_list:
                rospy.logwarn("Knockoff heard")
                self.__knock_off_detected = True
                
        

    def succeeded(self):
            return not self.__knock_off_detected == True

    def completed(self):
            #for now, someone else should tell us if this finished
            return False 



class AudioGuard(ActionMonitor):
    def get_name(self):
        return "AudioGuard"
    
    def __init__(self, update_models = False, sample_delay = 5, disjoint = True, soundstring = "", warning_channel_name = None):
        self.__knock_off_detected = False
        self.sample_delay = sample_delay
        self.disjoint = disjoint
        self.soundstring = sound_string
        ActionMonitor.__init__(self, update_models, warning_channel_name = warning_channel_name)


        
    def build_model(self):
        clanks = AudioDetectorModel([len(self.soundstring.split())] + [global_sound_dict[st] for st in self.soundstring.split()])
        clanks_predicate = []
        if self.disjoint:
            clanks_predicate = DisjointSetsModel(clanks)
        else:
            clanks_predicate = OverlappingSetsModel(clanks)
        top_level_model = clanks_predicate
        if self.sample_delay:
            top_level_model = SampleDelayModel(top_level_model, [1, self.sample_delay])

        sp = SensorPredicateManager(top_level_model, warning_channel_name = self.warning_channel_name)                
        
        self.sensor_predicate_list.append(sp)
        

    def handle_predicate_failures(self, failure_list):
            for f in failure_list:
                rospy.logwarn("Knockoff heard")
                self.__knock_off_detected = True
                
        

    def succeeded(self):
            return not self.__knock_off_detected == True

    def completed(self):
            #for now, someone else should tell us if this finished
            return False 

        

class GuardedArmMotion(GuardedFTMotion, HandActionMonitor):
    def get_name(self):
        return "GuardedArmMotion"

    def __init__(self, world_direction, end_effector_transform, threshold):
        GuardedFTMotion.__init__(self, world_direction, end_effector_transform, threshold)
        self.knocked_over = False

    def build_model(self):
        #Set up FT
        GuardedFTMotion.build_model(self)
        #Set up static hand stuff
        StrainTholdModel = bind_to_factory(ThresholdDataModel, internal_params = np.array([1, 1000]))
        self.setup_finger_models(StrainFingerModel, model_list = [StrainTholdModel])

        TactileThresholdModel = bind_to_factory(ThresholdDataModel, internal_params = [24] + (np.zeros([24,1]).tolist()))
        self.setup_finger_models(TactileFingerModel,  model_list = [TactileThresholdModel])
        
        armstate = WamStateModel([2, pr_msgs.msg.WAMState.state_free, pr_msgs.msg.WAMState.state_moving])
        state_test = OverlappingSetsModel(armstate)

        sp = SensorPredicateManager(state_test)
        self.sensor_predicate_list.append(sp)
        #watch out for clanks
        clanks = AudioDetectorModel([3, global_sound_dict["pipe"], global_sound_dict["hammer"], global_sound_dict["stapler"]])
        clanks_predicate = DisjointSetsModel(clanks)
        sp2 = SensorPredicateManager(clanks_predicate)
        self.sensor_predicate_list.append(sp2)

    def handle_predicate_failures(self, failure_list):
        for f in failure_list:
            self.remove_predicate(f)
            if f.get_name().find('audio') > -1:
                self.knocked_over = True
        """
        cancel = rospy.ServiceProxy('/right/owd/CancelAllTrajectories', pr_msgs.srv.CancelAllTrajectories)

                try:
                cancel()
            except:
            rospy.logwarn("Trajectories: Failed to clear paused trajectories")            
                """
        
        self.close()

        def succeeded(self):
            return self.completed() and not self.timed_out() and not self.knocked_over


"""Watch for off axis torques about the axis perpendicular to the line between the fingers

"""

class DrillingActionMonitor(GuardedFTMotion):
    def get_name(self):
        return "DrillingAction"
    
    def __init__(self, torque_threshold, update_models = False):
        self.__threshold = torque_threshold
        self.__drill_failure_detected = False
        ActionMonitor.__init__(self, update_models)
        
    def build_model(self):
        #looking only at torques
        base_force = ForceTransducerModel([3] + (range(3,6)))

        torque_direction = [0.0 ,1.0, 0.0]        
        watched_torque_direction = np.dot(rot_mat , torque_direction)
        
        force_opposite_direction = AffineModel(base_force, [5] +[1,3] +  (watched_torque_direction.tolist()))
        force_threshold = AbsThresholdDataModel(force_opposite_direction, [1, self.__threshold])
        sp = SensorPredicateManager(force_threshold)
        clanks = AudioDetectorModel(["drill"])
        clanks_predicate = OverlappingSetsModel(clanks)
        sp2 = SensorPredicateManager(clanks_predicate)
        self.sensor_predicate_list.append(sp)
        self.sensor_predicate_list.append(sp2)
        

    def handle_predicate_failures(self, failure_list):
            for f in failure_list:
                rospy.logwarn("Drilling task appears to be failing: f.warn()")
                self.__drill_failure_detected = True
                
        

    def succeeded(self):
            return not self.__drill_failure_detected

    def completed(self):
            #for now, someone else should tell us if this finished
            return False 



def RegisterMonitor(global_data, monitor_name, monitor):
    if global_data.action_monitors.has_key(monitor_name):
        return 0, "RegisterMonitors: Cannot reuse monitor name.  Please unregister monitor"
    global_data.action_monitors[monitor_name] = monitor
    return 1, "done"

def UnregisterMonitor(global_data, monitor_name):
    if not global_data.action_monitors.has_key(monitor_name):
        return 0, "RegisterMonitors: Cannot unregister monitor - no such monitor name"
    reason = "not completed"
    if global_data.action_monitors[monitor_name].completed():
        reason = "completed"
    global_data.action_monitors[monitor_name].close()
    global_data.action_monitors.pop(monitor_name)
    return 1, reason


'''
Move hand service 
'''
def MoveHandSrv(movetype, movevalues):
    try:
        owd_movehand = rospy.ServiceProxy('/bhd/MoveHand', pr_msgs.srv.MoveHand)
        res = owd_movehand(movetype, movevalues)
        return 1
    except rospy.ServiceException, e:
        print "Service call to MoveHand failed: %s"%e
        return 0


def SetHandSpeed( jointSpeedArray, jointAccelerationLimit = 0.2):
    success = 0
    loginfo( "Hand speed: %s " % jointSpeedArray )
    try:
        success = 1
        SetSpeedSrv = rospy.ServiceProxy( '/bhd/SetSpeed', SetSpeed )
        res = SetSpeedSrv( jointSpeedArray, jointAccelerationLimit )
    except rospy.ServiceException, reason:
        logerr( "Service call to SetSpeed failed: %s" % reason )
    return success


def GetHandPosition():
    msg = rospy.wait_for_message("/right/bhd/handstate", pr_msgs.msg.BHState, timeout = .5)
    return msg.positions
