#!/usr/bin/env python

# Imports for ROS
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Global publisher
publisher = None

#variables
LOW_DISTANCE_VALUE = 1.0
LOW_DISTANCE_COUNT_MAX = 2
#forward_distance = 0.0
is_forward_clear = False

def lidar_data_handler(data):
	#global forward_distance
	global is_forward_clear

	# Store ranges
	ranges = data.ranges
	# Create array for chosen values
	values = [None] * 50

	# Fill chosen values array
	for i in range(0, 50):
		values[i] = ranges[155 + i]

	# Check if forward is clear and publish its state
	currentLowDistCount = 0
	for i in range(0, 50):
		if (values[i] != float('Inf')) and (values[i] <= LOW_DISTANCE_VALUE):
			currentLowDistCount += 1
	if currentLowDistCount >= LOW_DISTANCE_COUNT_MAX:
		is_forward_clear = False
	else:
		is_forward_clear = True

	publisher.publish(is_forward_clear)

	# Count how many inf and lowDist
	# valueSum = 0.0;
	# infCount = 0
	# lowDistCount = 0
	# for i in range(0, 50):
	# 	if values[i] == float('Inf'):
	# 		infCount += 1
	# 	else:
	# 		valueSum += values[i]
	# 		if values[i] <= LOW_DISTANCE_VALUE:
	# 			lowDistCount += 1
	# if lowDistCount >= LOW_DISTANCE_COUNT_MAX:
	# 	is_forward_clear = False
	# else:
	# 	is_forward_clear = True
	# forward_distance = valueSum / (50 - infCount)
	#print("infCount", infCount, "lowDistCount", lowDistCount, "forward_distance", forward_distance, "forward_clear", forward_clear)

def lidar_data_receiver_setup():
	global publisher
	publisher = rospy.Publisher('lucy/lidar_forward_clear', Bool, queue_size=10)
	rospy.init_node('lidar_controller', anonymous=True)
	rospy.Subscriber('/scan', LaserScan, lidar_data_handler)

	rospy.spin()
		
	print("Stopped lidar_controller")

if __name__ == '__main__':
	try:
		print("Starting lidar_controller")
		lidar_data_receiver_setup()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
