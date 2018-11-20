#!/usr/bin/env python
# create the occupancy grid message type
# import ROS related
import rospy
from nav_msgs.msg import OccupancyGrid
import tf
# import for mathematical manipulation
import numpy
import math
# for filtering
from scipy.signal import medfilt

# to publish on exit
class final_publisher(object):
	def __init__(self):
		# create this node
		rospy.init_node('map_publisher')
		# some params for map
		self.width = 2000
		self.height = 2000
		self.resolution = 0.02

		# just a map in which i'll do a median filter
		self.map_no_filtered = self.create_map_no(self.width,self.height,self.resolution)
		self.map_pub_filtered = rospy.Publisher('/map3', OccupancyGrid, queue_size = 0)
		rospy.Subscriber('/map2', OccupancyGrid, self.mapCallback)

	# ---------------- call backs for data ------------------
	def mapCallback(self, msg):
		# set the laser values
		self.map_no_filtered.data=msg.data


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

		return mapp

	# ------------------------------------------------------
	# filtering related functions, aka, after processing!
	def filtermap(self):
		while not rospy.is_shutdown():
			# apply median filter at log odds - because
			self.map_no_filtered.data = medfilt(self.map_no_filtered.data)
			self.map_pub_filtered.publish(self.map_no_filtered)
			print("published with filter")
			rospy.sleep(30)

# our main to create the object and run the algorithm
def main():
	mypublish = final_publisher()
	mypublish.filtermap()


# does this line also go in if we're a ROS node?
if __name__ == '__main__':
	main()
