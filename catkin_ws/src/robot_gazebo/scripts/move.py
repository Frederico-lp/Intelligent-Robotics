#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, cos, sin, inf
from sensor_msgs.msg import LaserScan
import numpy as np

'''
STATES

0	Initial (wait for scan)
1	Go down
2
3
4
5
6
7
8
9
10
11
12
'''

class TurtleBot:
 
	def __init__(self):
		rospy.init_node('turtlebot_node', anonymous=True)
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)		
		self.originaltime = rospy.Time.now().to_sec()
		self.state = 0
		self.d = 1
		self.angular = 0
		self.distance = 0
		self.min_dist = 0.5
		self.max_dist = 1
		self.follow_dir = -1
		self.criterium = False
		self.stop = False
		rospy.on_shutdown(self.shutdown)


	# Publish linear and angular velocities for for finding wall
	def find_wall(self):
		velocity = Twist()
		velocity.linear.x = 0.3
		velocity.angular.z = 0
		return velocity
		
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


		'''
		Function: move_ahead:  This function publishes linear and angular velocities for moving straight.
		'''


	def move_ahead(self):
		velocity = Twist()
		velocity.linear.x = 0.3
		velocity.angular.z = 0
		return velocity


		'''
		Function: move_diag_right:  This function publishes linear and angular velocities for moving diagonally right.
		'''


	def move_diag_right(self):
		velocity = Twist()
		velocity.linear.x = 0.1
		velocity.angular.z = -0.3
		return velocity


		'''
		Function: move_diag_left:  This function publishes linear and angular velocities for moving diagonally left.
		'''


	def move_diag_left(self):
		velocity = Twist()
		velocity.linear.x = 0.1
		velocity.angular.z = 0.3
		return velocity
	
		
	def move(self):
		rospy.Subscriber('/scan', LaserScan, self.callback_laser)
		
		while not rospy.is_shutdown():
			vel_msg = Twist()	

			if self.state == 0:
				vel_msg = self.find_wall()
			elif self.state == 1:
				vel_msg = self.turn_right()
			elif self.state == 2:
				vel_msg = self.move_ahead()
			elif self.state == 3:
				vel_msg = self.turn_left()
			elif self.state == 4:
				vel_msg = self.move_diag_right()
			elif self.state == 5:
				vel_msg = self.move_diag_left()
			else:
				rospy.logerr('Unknown state!')

	

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		
	def shutdown(self):
		self.velocity_publisher.publish(Twist())
		rospy.sleep(1)
		
	def callback_laser(self, msg):
		laser_range = np.array(msg.ranges)
		#self.distance = np.min(laser_range)

		front = min(laser_range[70:110])
		left = min(laser_range[0:40])
		right = min(laser_range[140:180])

		velocity = Twist()

		if front > self.max_dist and left > self.max_dist and right > self.max_dist:
			self.state = 0
		elif self.follow_dir == -1:
			if left < self.max_dist:
				self.state = 1
				self.follow_dir = 0
				rospy.loginfo("following left wall")

			elif right < self.max_dist:
				self.state = 3
				self.follow_dir = 1
				rospy.loginfo("follow right wall")

			else:
				self.state = 2

		if self.follow_dir == 0: #follow left wall
			if left > self.max_dist and front > self.min_dist:
				self.state = 4
			elif left < self.max_dist and front > self.min_dist:
				self.state = 2
			elif left < self.max_dist and front < self.min_dist:
				self.state = 3
			else:
				rospy.loginfo("follow left wall not running")

		if self.follow_dir == 0: #follow right wall
			if right > self.max_dist and front > self.min_dist:
				self.state = 4
			elif right < self.max_dist and front > self.min_dist:
				self.state = 2
			elif right < self.max_dist and front < self.min_dist:
				self.state = 3
			else:
				rospy.loginfo("follow right wall not running")

			
		
		

if __name__ == '__main__':
	try:
		turtle = TurtleBot()
		turtle.move()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
