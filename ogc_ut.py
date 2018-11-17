#!/usr/bin/env python
# create the occupancy grid message type
# import ROS related
import rospy
from nav_msgs.msg import OccupancyGrid
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
# import for mathematical manipulation
import numpy
import math
# semaphores for concurrent processes
import threading

class occupancy_grid(object):
	def __init__(self):
		# create this node
		rospy.init_node('mapping_occupancy_grid')
		# set subscribes and publishes (check topic names)
		rospy.Subscriber("/scan", LaserScan, self.LaserCallback)
		rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.PositionCallback)
		self.map_pub = rospy.Publisher('/map2',OccupancyGrid, queue_size=0)
		# laser params (start empty)
		self.max_range= 5
	   	self.min_range= 0.02
		self.angle_min= None
		self.angle_max= None
		self.angle_increment= None
		self.ranges= None
		# pose params (start empty)
		self.pos_x = None
		self.pos_y = None
		self.pos_yaw = None
		self.offset = 5.0 # start at pose 5.0, 5.0 meters
		# some params for map
		self.width = 2000
		self.height = 2000
		self.resolution = 0.02
		# map in normal odds and map in log odds
		self.map_no = self.create_map_no(self.width,self.height,self.resolution)
		self.map_lo = numpy.zeros((self.width,self.height))
		print("Initialized")


	# ---------------- call backs for data ------------------
	def LaserCallback(self, msg):
		# set the laser values
		self.max_range=msg.range_max - 0.5 # account for error?
	   	self.min_range=msg.range_min + 0.5 # why not?
		self.angle_min=msg.angle_min
		self.angle_max=msg.angle_max
		self.angle_increment=msg.angle_increment
		self.ranges=msg.ranges

	def PositionCallback(self,msg):
		# set position values
		#position and orientation for 3d spaces (6 variables)
		quartenion= [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

		#get it on the object for 2d (3 variables)
		self.pos_x = msg.pose.pose.position.x
		self.pos_y = msg.pose.pose.position.y
		self.pos_yaw = yaw

	# ------------------------------------------------------

	# ---------------- mapp creation related ----------------
	def create_map_no(self, width, height, resolution):
		# time to set initial values
		mapp= OccupancyGrid()
		mapp.info.resolution=resolution
		mapp.info.width=width
		mapp.info.height=height     
		mapp.info.origin.position.x=-5
		mapp.info.origin.position.y=-5
		mapp.info.origin.position.z=0
		mapp.info.origin.orientation.x=0
		mapp.info.origin.orientation.y=0
		mapp.info.origin.orientation.z=0
		mapp.info.origin.orientation.w=0
		mapp.data=[]

		# start map at zeros
		for i in range (0,width*height):
			mapp.data.append(50)

		# publishing to see initial content
		self.map_pub.publish(mapp)
		return mapp


	# ------------------------------------------------------

	# ---------------- line drawing related ----------------
	def plotLineLow(self, x0,y0, x1,y1, lupdate):
		#bayers log odds
		l0 = math.log(0.5/0.5) #0
		lupdate_aux = lupdate
		#bresenham
		dx = x1 - x0
		dy = y1 - y0
		yi = 1
		xi = 1
		#the incrementations xi yi depend on the sign
		if dy < 0:
			yi = -1
			dy = -dy
		if dx < 0:
			xi = -1
		D = 2*dy - dx
		y = y0

		#walk according to the inclination
		for x in range(x0, x1+1,xi):
			self.map_lo[x][y] = self.map_lo[x][y] - l0 + lupdate_aux
			self.map_no.data[x + self.width * y] = int(round(100*(1 - 1/(1+math.exp(self.map_lo[x][y])))))
			if D > 0:
			   y = y + yi
			   D = D - 2*dx
			D = D + 2*dy
		
	def plotLineHigh(self, x0,y0, x1,y1, lupdate):
		#bayers log odds
		l0 = math.log(0.5/0.5) #0
		lupdate_aux = lupdate
		#bresenham
		dx = x1 - x0
		dy = y1 - y0
		xi = 1
		yi = 1
		#the incrementations xi yi depend on the sign
		if dx < 0:
			xi = -1
			dx = -dx
		if dy < 0:
			yi = 1
		D = 2*dx - dy
		x = x0

		#walk according to the inclination
		for y in range(y0, y1+1, yi):
			self.map_lo[x][y] = self.map_lo[x][y] - l0 + lupdate_aux
			self.map_no.data[x + self.width * y] = int(round(100*(1 - 1/(1+math.exp(self.map_lo[x][y])))))
			if D > 0:
				x = x + xi
				D = D - 2*dy
			D = D + 2*dx
		

	#draw our line with the log odds
	def bresenham_line(self,x0, y0, x1, y1, lupdate):
		#check octants
		if abs(y1 - y0) < abs(x1 - x0):
			if x0 > x1:
			  self.plotLineLow(x1, y1, x0, y0, lupdate)
			else:
			  self.plotLineLow(x0, y0, x1, y1, lupdate)
		else:
			if y0 > y1:
			  self.plotLineHigh(x1, y1, x0, y0, lupdate)
			else:
			  self.plotLineHigh(x0, y0, x1, y1, lupdate)


	# ------------------------------------------------------

	# ----------------------map filler----------------------

	# returns destination point in pixels
	def calculate_hit(self, x0, y0, distance, rotation):
		# initial variables of log odds
		lupdate = 0
		lfree = math.log(0.3/(1-0.3))
		locc = math.log(0.7/(1-0.7))
		lunk = math.log(0.5/(1-0.5))
		# according to documentation the error is given by
		if distance > 0 and distance < 1:
			laser_error =  0.03
		else:
			laser_error = distance * 0.03
		
		# assuming distance in meters translate it to pixels
		# assuming bounce of laser error to mininum value
		x1 = int(x0+round((distance - 0.5*laser_error)*(math.cos(rotation))*(1/self.resolution)))
		y1 = int(y0+round((distance - 0.5*laser_error)*(math.sin(rotation))*(1/self.resolution)))
		# assuming bounce of laser error to maximum value
		x2 = int(x0+round((distance + 0.5*laser_error)*(math.cos(rotation))*(1/self.resolution)))
		y2 = int(y0+round((distance + 0.5*laser_error)*(math.sin(rotation))*(1/self.resolution)))
		# point after the hit, but within laser range
		x3 = int(x0+round((self.max_range)*(math.cos(rotation))*(1/self.resolution)))
		y3 = int(y0+round((self.max_range)*(math.sin(rotation))*(1/self.resolution)))
		#free part of the line
		lupdate = lfree
		self.bresenham_line(x0, y0, x1, y1, lupdate)
		#occupied part of the line
		lupdate = locc
		self.bresenham_line(x1, y1, x2, y2, lupdate)
		#get it to be unknonw
		lupdate = lunk
		self.bresenham_line(x2,y2, x3,y3, lupdate)
		

	# ----------------- OUR PROGRAM MASTER -----------------
	# calls all other functions to update the og
	def occupancy_grider(self):
		# some local variables
		j = 1
		landing_points = []
		# it's supposed to run on a loop
		while not rospy.is_shutdown():
			# stuck until we get data
			if None in (self.pos_x, self.ranges):
				rospy.sleep(0.1)
				continue

			# translate meters to pixels of position
			pos_x = int(round((self.pos_x + self.offset) / self.resolution))
			pos_y = int(round((self.pos_y + self.offset) / self.resolution))
			print("pos_x ", pos_x, "pos_y", pos_y, "iteration: ", j)

			# for each range
			for i in range(0, len(self.ranges)):
				# do not let the callbacks steal the value!
				range_aux = self.ranges[i]
				# check if it's a valid value
				if not numpy.isnan(range_aux) and range_aux <= self.max_range and range_aux >= self.min_range:
					# calculate laser lines in the map
					rotation = self.pos_yaw + self.angle_min + self.angle_increment*i
					self.calculate_hit(pos_x, pos_y, range_aux, rotation)
				

			# publish in the end of x iterations, because
			if j > 500:
				self.map_pub.publish(self.map_no)
				print("Published!")				
				j = 1
			j = j + 1
		



# our main to create the object and run the algorithm
def main():
	myobject = occupancy_grid()
	myobject.occupancy_grider()


# does this line also go in if we're a ROS node?
if __name__ == '__main__':
	main()
