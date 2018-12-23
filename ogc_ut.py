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
# for syncro
import message_filters
# others
import time

class occupancy_grid(object):
	def __init__(self):
		# create this node
		rospy.init_node('mapping_occupancy_grid')
		# set subscribes and publishes (check topic names)
		lasersub = message_filters.Subscriber("/scan", LaserScan)
		posesub = message_filters.Subscriber("/amcl_pose",PoseWithCovarianceStamped)
		sincronizer = message_filters.TimeSynchronizer([lasersub, posesub], 10)
		sincronizer.registerCallback(self.dataCallback)
		self.map_pub = rospy.Publisher('/map2',OccupancyGrid, queue_size=0)
		# laser params (start empty)
		self.max_range= None
	   	self.min_range= None
		self.angle_min= None
		self.angle_max= None
		self.angle_increment= None
		self.ranges= None
		# pose params (start empty)
		self.cov = None
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
	def dataCallback(self, msg_laser, msg_pose):
		# set the laser values
		self.max_range=msg_laser.range_max -0.5
	   	self.min_range=msg_laser.range_min +0.5
		self.angle_min=msg_laser.angle_min
		self.angle_max=msg_laser.angle_max
		self.angle_increment=msg_laser.angle_increment
		self.ranges=msg_laser.ranges
		
		# set position values
		#covariance
		self.cov = msg_pose.pose.covariance
		#orientation for 3d spaces (4 variables)
		self.quartenion= [msg_pose.pose.pose.orientation.x, msg_pose.pose.pose.orientation.y, msg_pose.pose.pose.orientation.z, msg_pose.pose.pose.orientation.w]
		#position for 2d (2 variables)
		self.pos_x = msg_pose.pose.pose.position.x
		self.pos_y = msg_pose.pose.pose.position.y

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
		l0 = 0
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
			try:
				self.map_no.data[x + self.width * y] = int(round(100*(1 - 1/(1+math.exp(self.map_lo[x][y])))))
			except:
				pass
			if D > 0:
			   y = y + yi
			   D = D - 2*dx
			D = D + 2*dy
		
	def plotLineHigh(self, x0,y0, x1,y1, lupdate):
		#bayers log odds
		l0 = 0
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
			try:
				self.map_no.data[x + self.width * y] = int(round(100*(1 - 1/(1+math.exp(self.map_lo[x][y])))))
			except:
				pass
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

		# free part of the line
		lupdate = lfree
		self.bresenham_line(x0, y0, x1, y1, lupdate)
		# occupied part of the line WITH UNIFORM DISTRIBUTION (KINDA WRONG FOR NOW)
		# lupdate = locc/numpy.linalg.norm([x1-x2,y1-y2])
		lupdate = locc
		self.bresenham_line(x1, y1, x2, y2, lupdate)
		

	# ----------------- OUR PROGRAM MASTER -----------------
	# calls all other functions to update the og
	def occupancy_grider(self):
		# some local variables
		(j,p) = [1,1]
		file = open("/home/yourself/Desktop/basement.txt", "w")
		# it's supposed to run on a loop
		while not rospy.is_shutdown():
			# stuck until we get data
			if None in (self.pos_x, self.ranges):
				rospy.sleep(0.01)
				continue

			# tic
			tic = time.time()

			# steal all unstable variables from object to not interfere in the calcule
			pos_xm = self.pos_x
			pos_ym = self.pos_y
			quartenion = self.quartenion
			scan_ranges = list(self.ranges)

			# translate meters to pixels of position
			pos_x = int(round((pos_xm + self.offset) / self.resolution))
			pos_y = int(round((pos_ym + self.offset) / self.resolution))
			# translate quartenion to roll, pitch, yaw. we only need yaw			
			(roll, pitch, pos_yaw) = tf.transformations.euler_from_quaternion(quartenion)
			
			# print check
			# print("pos_x ", pos_x, "pos_y", pos_y,"pos_yaw", round(pos_yaw,4), "j: ", j)

			# for each range
			for i in range(0, len(scan_ranges)):
				# check if it's a valid value
				if not numpy.isnan(scan_ranges[i]) and scan_ranges[i] <= self.max_range and scan_ranges[i] >= self.min_range:
					# calculate laser lines in the map
					rotation = pos_yaw + self.angle_min + self.angle_increment*i
					self.calculate_hit(pos_x, pos_y, scan_ranges[i], rotation)
				
			# toc
			toc = time.time() - tic
			file.write(str(toc) + "\n")

			# publish in the end of x iterations, because
			if j > 50:
				self.map_pub.publish(self.map_no)		
				j = 1
			j = j + 1

		file.close()



# our main to create the object and run the algorithm
def main():
	mymap = occupancy_grid()
	mymap.occupancy_grider()


# does this line also go in if we're a ROS node?
if __name__ == '__main__':
	main()
