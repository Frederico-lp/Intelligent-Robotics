#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, cos, sin, inf
from sensor_msgs.msg import LaserScan
import numpy as np
from enum import Enum


class State(Enum):
	MOVE_AHEAD = 0
	MOVE_BACKWARDS = -1
	TURN_LEFT = 1
	TURN_RIGHT = 2
	MOVE_DIAG_LEFT = 3
	MOVE_DIAG_RIGHT = 4


class TurtleBot:
 
	def __init__(self):
		rospy.init_node('turtlebot_node', anonymous=True)
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)		
		self.originaltime = rospy.Time.now().to_sec()
		self.state = 0

		self.min_dist = 0.15
		self.max_dist = 0.7

		self.target_dist = 0.3

		rospy.on_shutdown(self.shutdown)

		
	def turn_left(self):
		velocity = Twist()
		velocity.linear.x = 0
		velocity.angular.z = 0.3
		return velocity
	
	def turn_right(self):
		velocity = Twist()
		velocity.linear.x = 0
		velocity.angular.z = -0.3
		return velocity

	def move_ahead(self):
		velocity = Twist()
		velocity.linear.x = 0.1
		velocity.angular.z = 0
		return velocity

	def move_diag_right(self):
		velocity = Twist()
		velocity.linear.x = 0.1
		velocity.angular.z = -0.7
		return velocity

	def move_diag_left(self):
		velocity = Twist()
		velocity.linear.x = 0.1
		velocity.angular.z = 0.7
		return velocity

	def move_backwards(self):
		velocity = Twist()
		velocity.linear.x = -0.1
		return velocity
	
	def stop(self):
		return Twist()
		
	def move(self):
		rospy.Subscriber('/scan', LaserScan, self.callback_laser)
		
		while not rospy.is_shutdown():
			vel_msg = Twist()	

			vel_msg = self.move_ahead()

			if self.state == 0:
				vel_msg = self.move_ahead()
			elif self.state == 1:
				vel_msg = self.turn_left()
			elif self.state == 2:
				vel_msg = self.turn_right()
			elif self.state == 3:
			 	vel_msg = self.move_diag_left()
			elif self.state == 4:
			 	vel_msg = self.move_diag_right()
			elif self.state == -1:
				vel_msg = self.move_backwards()
			elif self.state == -2:
				vel_msg = self.stop()
			else:
				rospy.logerr('Unknown state!')

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		
	def shutdown(self):
		self.velocity_publisher.publish(self.stop())
		rospy.loginfo("Shutting down")
		rospy.sleep(1)
		
	def callback_laser(self, msg):
	
		laser_range = np.array(msg.ranges)
		laser_range = [value if value <= self.max_dist else inf for value in laser_range]

		left_dist = min(laser_range[60:120])
		
		front_dist = min(laser_range[0:60])

		right_dist = min(laser_range[240:300])

		if all(value >= inf for value in laser_range):
			self.state = -2 # STOP
			rospy.loginfo("Stopped")
		elif (front_dist < self.target_dist):
			self.state = -1
		elif ((left_dist <= front_dist and left_dist < right_dist)
			or (front_dist < right_dist or front_dist < self.target_dist)
			or (front_dist < self.target_dist and right_dist < self.target_dist)
			or (right_dist < self.min_dist)):
			self.state = 1
		elif (right_dist < inf):
			if (right_dist < self.target_dist):
				self.state = 0
			else:
				self.state = 4
		else:
			# There is something on the back
			self.state = 1


		if (self.state != -2):
			print(left_dist, front_dist, right_dist)
			print(self.state)
		

		# if (front_dist < right_dist):
		# 	self.state = 1
		# elif (right_dist < front_dist):
		# 	if (right_dist < self.min_dist):
		# 		self.state = 2 
		# 	elif (right_dist > self.max_dist):
		# 		self.state = 1
		# 	else:
		# 		self.state = 0			
		
		

if __name__ == '__main__':
	try:
		turtle = TurtleBot()
		turtle.move()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass