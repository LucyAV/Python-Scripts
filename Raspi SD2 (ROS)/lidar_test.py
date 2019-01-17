#!/usr/bin/env python

# Imports for ROS
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

#variables
LOW_DISTANCE_VALUE = 1.5
LOW_DISTANCE_COUNT_MAX = 2

def lidar_data_handler(data):
	# Store ranges
	ranges = data.ranges
	# Create array for chosen values
	print(ranges[180])
	print(len(ranges))

def lidar_data_receiver_setup():
	rospy.init_node('lidar_test', anonymous=True)
	rospy.Subscriber('/scan', LaserScan, lidar_data_handler)

	rospy.spin()
		
	print("Stopped lidar_test")

if __name__ == '__main__':
	try:
		print("Starting lidar_test")
		lidar_data_receiver_setup()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
