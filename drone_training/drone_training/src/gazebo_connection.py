#!/usr/bin/env python3
'''
Created on Mar 20, 2022
@author Vincent

Establishes connection to necessary gazebo servers. 
This file is Updated version of the GazeboConnection used in https://bitbucket.org/theconstructcore/drone_training.git by Ricardo Tellez.
Updated interpreter path for Python 3 and resolved exception errors.
'''
import rospy
from std_srvs.srv import Empty

class GazeboConnection():
'''
Gazebo connection class requires connections to the gazebo servers to unpause, pause and reset the gazebo simulation after each episdode.
'''
    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    '''
    Pause simulation.
    Send Empty message to /gazebo/pause_physics topic.
    '''
    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")
    '''
    Unpause simulation.
    Send Empty message to /gazebo/upause_physics topic.
    '''    
    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print ("/gazebo/unpause_physics service call failed")
    '''
    Reset simulation.
    Send Empty message to /gazebo/reset_simulation topic.
    '''    
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")
