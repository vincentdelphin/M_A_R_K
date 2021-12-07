#! /usr/bin/env python
import rospy
import actionlib
import time
from actionlib.msg import TestFeedback, TestAction, TestResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class movement_class(object):

    def __init__(self):
        self.ctrl_c = False
        self._as = actionlib.SimpleActionServer("proof_of_concept_demo_service", TestAction, self.goal_callback, False)
        self.rate = rospy.Rate(10)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._move_message = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size = 1)
        self._takeoff_message = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size = 1) 
        self._as.start()
        self._result = TestResult()
        self._feedback = TestFeedback()

    def strafe_forward(self):
        rospy.loginfo("Moving forward ...")
        self._move_message.linear.x = 1.0
        self._move_message.angular.z = 0
        self.publish_into_cmd(self._move_message)

    def turn(self):
        rospy.loginfo("Turning ...")
        self._move_message.linear.x = 0.0
        self._move_message.angular.z = 1.0
        self.publish_into_cmd(self._move_message)


    def strafe_backward(self):
        rospy.loginfo("Strafing backward...")
        self._move_message.linear.x = -1.0
        self._move_message.linear.y = 0
        self._move_message.linear.z = 0
        self._move_message.angular.z = 0
        self.publish_into_cmd(self._move_message)

    
    def strafe_left(self):
        rospy.loginfo("Strafing left ...")
        self._move_message.linear.x = 0
        self._move_message.linear.y = 1
        self._move_message.linear.z = 0
        self._move_message.angular.z = 0
        self.publish_into_cmd(self._move_message)

        
    def strafe_right(self):
        rospy.loginfo("Strafing right ...")
        self._move_message.linear.x = 0
        self._move_message.linear.y = -1
        self._move_message.linear.z = 0
        self._move_message.angular.z = 0
        self.publish_into_cmd(self._move_message)
    

    def takeoff(self):
        i = 0
        while not i == 3:
            self._pub_takeoff.publish(self._takeoff_message)
            rospy.loginfo("Taking off ...")
            time.sleep(1)
            i = i + 1
    
    def stop(self):
        rospy.loginfo("Stopping ...")
        self._move_message.linear.x = 0
        self._move_message.angular.z = 0
        self.publish_into_cmd(self._move_message)
        time.sleep(1)

    def publish_into_cmd(self, movement):
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(movement)
                rospy.loginfo("Publishing to cmd_vel topic ...")
                break
            else:
                self.rate.sleep()


    def land(self):
        i = 0
        while not i == 3:
            self._pub_land.publish(self._takeoff_message)
            rospy.loginfo("Landing ...")
            time.sleep(1)
            i = i + 1
    

    def goal_callback(self, goal):
        r = rospy.Rate(1)
        success = True
        rospy.loginfo("Proof of concept movement proceeding ...")
        # MAKE M-A-R-K move foward, turn around, land, lift off and then return to his position.
        distance_moved = goal.goal

        counter = 0
            # Make M-A-R-K takeoff.
        while counter <= 2:

             # Check if cancelation of demo has been requested. If so, then land drone.
            if self._as.is_preempt_requested():
                rospy.loginfo('The proof of concept demo has been cancelled.')
                self.land()
                self._as.set_preempted()
                success = False
                break

            # Make M-A-R-K takeoff.
            self.takeoff()
            self.stop()
            time.sleep(3)
             # Move M-A-R-K forward, strafing forward for as long as the user passes through as a goal.
            self.strafe_forward()
            time.sleep(distance_moved)
            self.stop()
            time.sleep(3)
             # Rotate M-A-R-K to the left for 3.5 seconds (roughly 180 degrees).
            self.turn()
            time.sleep(3.5)

            # Land M-A-R-K and wait for 5 seconds
            self.stop()
            self.land()
            time.sleep(3)
            # Send feedback message
            self._feedback.feedback = "Completed maneuver."
            self._as.publish_feedback(self._feedback)
            r.sleep()

            counter += 1

        self._result.result = "Completed Mission"
        # Send result message
        if success:
            rospy.loginfo('Successful mission!')
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('Driver')
    movement_class()
    rospy.spin()