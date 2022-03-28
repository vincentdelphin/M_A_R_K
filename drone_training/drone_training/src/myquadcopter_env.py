#!/usr/bin/env python3
'''
Created on Mar 20, 2022
@author Vincent
Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
Based on many other examples around Internet

This file is Updated version of the quadcopter environment used in https://bitbucket.org/theconstructcore/drone_training.git by Ricardo Tellez
Updated interpreter path for Python 3 and added pydoc
'''
import gym
import rospy
import time
import numpy as np
import tf
import time
from gym import utils, spaces
from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyTopicMsg
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection

#register the training environment in the gym as an available one
reg = register(
    id='QuadcopterLiveShow-v0',
    entry_point='myquadcopter_env:QuadCopterEnv',
    max_episode_steps=100,
    )


class QuadCopterEnv(gym.Env):
    '''
    Intialisation of environment including required topics to publish actions to such as /cmd_vel, /drone/takeoff etc
    Param server provides desired pose in the (x,y,z) coordinate axis
    '''
    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.takeoff_pub = rospy.Publisher('/drone/takeoff', EmptyTopicMsg, queue_size=0)
        
        # gets training parameters from param server
        self.speed_value = rospy.get_param("/speed_value")
        self.desired_pose = Pose()
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_altitude = rospy.get_param("/max_altitude")
        
        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        
        self.action_space = spaces.Discrete(5) #Forward,Left,Right,Up,Down
        self.reward_range = (-np.inf, np.inf)

        self._seed()
    '''
    Used to generate random number generator using numpy
    :returns: Random number for seeding simulation
    '''
    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    '''
    Reset gazebo environment including drone's starting position and desired pose in the (x,y,z) coordinate axis
    :returns: observation of environment in the x axis
    '''    
    # Resets the state of the environment and returns an initial observation.
    def reset(self):
        
        # 1st: resets the simulation to initial values
        self.gazebo.resetSim()

        # 2nd: Unpauses simulation
        self.gazebo.unpauseSim()

        # 3rd: resets the robot to initial conditions
        self.check_topic_publishers_connection()
        self.init_desired_pose()
        self.takeoff_sequence()

        # 4th: takes an observation of the initial condition of the robot
        data_pose, data_imu = self.take_observation()
        observation = [data_pose.position.x]
        
        # 5th: pauses simulation
        self.gazebo.pauseSim()

        return observation
    '''
    Executes action in simulation
    :param action: action decided by chosen algorithm in training script
    :return: state of agent in environment after performing action along with reward
    '''
    def step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        
        # 1st, we decide which velocity command corresponds
        vel_cmd = Twist()
        if action == 0: #FORWARD
            vel_cmd.linear.x = self.speed_value
            vel_cmd.angular.z = 0.0
        elif action == 1: #LEFT
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = self.speed_value
        elif action == 2: #RIGHT
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -self.speed_value
        elif action == 3: #Up
            vel_cmd.linear.z = self.speed_value
            vel_cmd.angular.z = 0.0
        elif action == 4: #Down
            vel_cmd.linear.z = -self.speed_value
            vel_cmd.angular.z = 0.0

        # Then we send the command to the robot and let it go
        # for running_step seconds
        self.gazebo.unpauseSim()
        self.vel_pub.publish(vel_cmd)
        time.sleep(self.running_step)
        data_pose, data_imu = self.take_observation()
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward,done = self.process_data(data_pose, data_imu)

        # Promote going forwards instead if turning
        if action == 0:
            reward += 100
        elif action == 1 or action == 2:
            reward -= 50
        elif action == 3:
            reward -= 150
        else:
            reward -= 50

        state = [data_pose.position.x]
        return state, reward, done, {}

    '''
    Provides drone observation from the drone's IMU (Inertial Measurement Unit) and drone/gt_pose topic.
    :return: current location of drone from both gt_pose topic and imu topic.
    '''
    def take_observation (self):
        data_pose = None
        while data_pose is None:
            try:
                data_pose = rospy.wait_for_message('/drone/gt_pose', Pose, timeout=5)
            except:
                rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose")

        data_imu = None
        while data_imu is None:
            try:
                data_imu = rospy.wait_for_message('/drone/imu', Imu, timeout=5)
            except:
                rospy.loginfo("Current drone imu not ready yet, retrying for getting robot imu")
        
        return data_pose, data_imu
    '''
    Calculates distance between two points in 3 dimensions. Used in generating distance from final position and desired position
    :param p_init: initial distance
    :param p_end: end distance
    :return: distance between two points
    '''
    def calculate_dist_between_two_Points(self,p_init,p_end):
        a = np.array((p_init.x ,p_init.y, p_init.z))
        b = np.array((p_end.x ,p_end.y, p_end.z))
        
        dist = np.linalg.norm(a-b)
        
        return dist

    '''
    Saves best distance travelled between initial point and desired destination
    '''
    def init_desired_pose(self):
        
        current_init_pose, imu = self.take_observation()
        
        self.best_dist = self.calculate_dist_between_two_Points(current_init_pose.position, self.desired_pose.position)
    
    '''
    Checks at a rate of 10 cycles per second if the connections between the takeoff and cmd_vel topics are connected.
    '''
    def check_topic_publishers_connection(self):
        me = self.takeoff_pub.get_num_connections()
        print(me)
        rate = rospy.Rate(10) # 10hz
        while(self.takeoff_pub.get_num_connections() == 0):
            rospy.loginfo("No susbribers to Takeoff yet so we wait and try again")
            rate.sleep();
        rospy.loginfo("Takeoff Publisher Connected")

        while(self.vel_pub.get_num_connections() == 0):
            rospy.loginfo("No susbribers to Cmd_vel yet so we wait and try again")
            rate.sleep();
        rospy.loginfo("Cmd_vel Publisher Connected")
        
    '''
    Sends a Twist message type with 0 linear and angular velocities in the Z directions
    Essentially, recalibrates the cmd_vel topic
    '''
    def reset_cmd_vel_commands(self):
        # We send an empty null Twist
        vel_cmd = Twist()
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)

    '''
    Sends empty message to /drone/take_off topic for drone to leave the ground before performing actions to reach desired pose
    :param seconds_taking_off: duration of taking off in the z axis
    '''
    def takeoff_sequence(self, seconds_taking_off=1):
        # Before taking off be sure that cmd_vel value there is is null to avoid drifts
        self.reset_cmd_vel_commands()
        
        takeoff_msg = EmptyTopicMsg()
        rospy.loginfo( "Taking-Off Start")
        self.takeoff_pub.publish(takeoff_msg)
        time.sleep(seconds_taking_off)
        rospy.loginfo( "Taking-Off sequence completed")
        
    '''
    Calculates reward dependent on if the distance travelled in the episode is less/more than the best distance to the desired pose achieved in a previous episode.
    :param current_pose: present position in time
    :return: reward
    '''
    def improved_distance_reward(self, current_pose):
        current_dist = self.calculate_dist_between_two_Points(current_pose.position, self.desired_pose.position)
        #rospy.loginfo("Calculated Distance = "+str(current_dist))
        
        if current_dist < self.best_dist:
            reward = 100
            self.best_dist = current_dist
        elif current_dist == self.best_dist:
            reward = 0
        else:
            reward = -100
            #print "Made Distance bigger= "+str(self.best_dist)
        
        return reward
    '''
    Considers if the altitude and orbital inclination reached by the drone breach the specified range
    If drone exceeds the ranges, then the episode is finished and negative reward is given
    :param data_position: present position in time used to find altitude
    :param data_imu: present position in time used to find orientation
    :return: reward
    '''    
    def process_data(self, data_position, data_imu):

        done = False
        
        euler = tf.transformations.euler_from_quaternion([data_imu.orientation.x,data_imu.orientation.y,data_imu.orientation.z,data_imu.orientation.w])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        pitch_bad = not(-self.max_incl < pitch < self.max_incl)
        roll_bad = not(-self.max_incl < roll < self.max_incl)
        altitude_bad = data_position.position.z > self.max_altitude

        if altitude_bad or pitch_bad or roll_bad:
            rospy.loginfo ("(Drone flight status is wrong) >>> ("+str(altitude_bad)+","+str(pitch_bad)+","+str(roll_bad)+")")
            done = True
            reward = -200
        else:
            reward = self.improved_distance_reward(data_position)

        return reward,done
