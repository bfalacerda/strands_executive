#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import *
import ros_datacentre.util as dc_util
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_datacentre.message_store import MessageStoreProxy
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class AbstractTaskExecutor(object):

    # These can be implemented by sub classes to provide hooks into the execution system

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        pass

    def pause_execution(self):
        """ Called when overall execution should pause """
        pass

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        pass

    def task_succeeded(self, task):
        """ Called when the given task has completed execution successfully """
        self.task_complete(task)

    def task_failed(self, task):
        """ Called when the given task has completed execution but failed """
        self.task_complete(task)


    def task_demanded(self, previously_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
        pass

    def __init__(self):
        self.task_counter = 1
        self.msg_store = MessageStoreProxy() 
        self.executing = False
        self.active_task = None
        self.active_task_id = Task.NO_TASK
        self.nav_client = None

        

    def advertise_services(self):
        """
        Adverstise ROS services. Only call at the end of constructor to avoid calls during construction.
        """
        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)                
                rospy.Service("/task_executor/" + attr[:-8], service.type, service)


    def get_task_types(self, action_name):
        """ 
        Returns the type string related to the action string provided.
        """
        rospy.logdebug("task action provided: %s", action_name)
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                return (type[:-8], type[:-14] + 'Goal')
        raise RuntimeError('No action associated with topic: %s'% action_name)


    def execute_task(self, task):
        self.active_task = task
        self.active_task_id = task.task_id               
        if self.active_task.start_node_id != '':                    
            self.start_task_navigation()
        elif self.active_task.action != '':                    
            self.start_task_action()
        else:
            rospy.logwarn('Provided task had no start_node_id or action %s' % self.active_task)
            self.active_task = None
            self.active_task_id = Task.NO_TASK


    def cancel_active_task(self, event):
        """ Cancel any active nav or task action """
        rospy.logwarn("Cancelling task that has overrun %s" % self.active_task)
        if self.nav_client and self.nav_client.get_state() == GoalStatus.ACTIVE:
            self.nav_client.cancel_goal()
        if self.action_client and self.action_client.get_state() == GoalStatus.ACTIVE:
            self.action_client.cancel_goal()

    def start_task_action(self):

        rospy.loginfo('Starting to execute %s' % self.active_task.action)

        (action_string, goal_string) = self.get_task_types(self.active_task.action)
        action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
        goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

        self.action_client = actionlib.SimpleActionClient(self.active_task.action, action_clz)
        self.action_client.wait_for_server()

        argument_list = self.get_arguments(self.active_task.arguments)

        # print "ARGS:"
        # print argument_list

        goal = goal_clz(*argument_list)         

        rospy.logdebug('Sending goal to %s' % self.active_task.action)
        self.action_client.send_goal(goal, self.task_execution_complete_cb) 
        
        wiggle_room = rospy.Duration(5)
        # start a timer to kill off tasks that overrun
        self.timeout_timer = rospy.Timer(self.active_task.max_duration + wiggle_room, self.cancel_active_task, oneshot=True)

    def start_task_navigation(self):
        # handle delayed start up
        if self.nav_client == None:
            self.nav_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
            self.nav_client.wait_for_server()
            rospy.logdebug("Created action client")

        nav_goal = GotoNodeGoal(target = self.active_task.start_node_id)
        self.nav_client.send_goal(nav_goal, self.navigation_complete_cb)
        rospy.loginfo("navigating to %s" % nav_goal)

    def navigation_complete_cb(self, goal_status, result):

        # print self.nav_client.get_state()
        # print self.nav_client.get_result()

        if self.nav_client.get_state() == GoalStatus.SUCCEEDED and self.nav_client.get_result().success:

            rospy.loginfo('Navigation to %s succeeded' % self.active_task.start_node_id)        

            if self.active_task.action != '':                                        
                self.start_task_action()
            else:
                self.task_succeeded(self.active_task)
                self.active_task = None
                self.active_task_id = Task.NO_TASK
        else:
            rospy.loginfo('Navigation to %s failed' % self.active_task.start_node_id)        
            self.task_failed(self.active_task)
            self.active_task = None
            self.active_task_id = Task.NO_TASK



    def task_execution_complete_cb(self, goal_status, result):

        if self.action_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Execution of task %s succeeded' % self.active_task.task_id)        
            self.task_succeeded(self.active_task)
        else:
            rospy.loginfo('Execution of task %s failed' % self.active_task.task_id)        
            self.task_failed(self.active_task)
        self.timeout_timer.shutdown()
        self.active_task = None
        self.active_task_id = Task.NO_TASK


    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        req.task.task_id = self.task_counter
        self.task_counter += 1
        self.add_tasks([req.task])                
        return req.task.task_id
    add_task_ros_srv.type=AddTask


    def add_tasks_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        task_ids = []
        for task in req.tasks:
            task.task_id = self.task_counter
            task_ids.append(task.task_id)
            self.task_counter += 1

        self.add_tasks(req.tasks)        
        
        return [task_ids]
    add_tasks_ros_srv.type=AddTasks

    def demand_task_ros_srv(self, req):
        """
        Demand a the task from the execution framework.
        """
        req.task.task_id = self.task_counter
        self.task_counter += 1

        # stop anything else
        self.cancel_active_task(None)

        # save what /was/ executing (if any)
        previously_active_task = self.active_task

        # trigger demended execution
        self.execute_task(req.task)

        # and inform implementation to let it take action
        self.task_demanded(previously_active_task)                        

        return req.task.task_id
    demand_task_ros_srv.type=DemandTask


    def get_execution_status_ros_srv(self, req):
        return self.executing
    get_execution_status_ros_srv.type = GetExecutionStatus

    def set_execution_status_ros_srv(self, req):
        if self.executing and not req.status:
            rospy.logdebug("Pausing execution")
            self.pause_execution()
        elif not self.executing and req.status:
            rospy.logdebug("Starting execution")
            self.start_execution()
        previous = self.executing
        self.executing = req.status
        return previous
    set_execution_status_ros_srv.type = SetExecutionStatus


    def instantiate_from_string_pair(self, string_pair):
        if len(string_pair.first) == 0:
            return string_pair.second
        elif string_pair.first == Task.INT_TYPE:
            return int(string_pair.second)
        elif string_pair.first == Task.FLOAT_TYPE:
            return float(string_pair.second)     
        else:
            msg = self.msg_store.query_id(string_pair.second, string_pair.first)[0]
            # print msg
            if msg == None:
                raise RuntimeError("No matching object for id %s of type %s" % (string_pair.second, string_pair.first))
            return msg

    def get_arguments(self, argument_list):
        return map(self.instantiate_from_string_pair, argument_list)



