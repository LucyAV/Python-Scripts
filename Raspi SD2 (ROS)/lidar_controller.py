#!/usr/bin/env python

# Imports for ROS
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

LOW_DISTANCE_VALUE = 1.0
LOW_DISTANCE_COUNT = 2
forward_distance = 0.0
forward_clear = False

def lidar_data_handler(data):
	global forward_distance
	global forward_clear

	# Store ranges
	ranges = data.ranges
	# Create array for chosen values
	values = [None] * 50

	# Fill chosen values array
	for i in range(0, 25):
		values[i] = ranges[i]
	for i in range(25, 50):
		values[i] = ranges[310 + i]

	# Count how many inf and lowDist
	valueSum = 0.0;
	infCount = 0
	lowDistCount = 0
	for i in range(0, 50):
		if values[i] == float('Inf'):
			infCount += 1
		else:
			valueSum += values[i]
			if values[i] <= LOW_DISTANCE_VALUE:
				lowDistCount += 1
	if lowDistCount >= LOW_DISTANCE_COUNT:
		forward_clear = False
	else:
		forward_clear = True
	forward_distance = valueSum / (50 - infCount)
	#print("infCount", infCount, "lowDistCount", lowDistCount, "forward_distance", forward_distance, "forward_clear", forward_clear)

def lidar_data_receiver_setup():
	publisher = rospy.Publisher('lucy/lidar_forward_clear', Bool, queue_size=10)
	rospy.init_node('lidar_controller', anonymous=True)
	rospy.Subscriber('/scan', LaserScan, lidar_data_handler)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		publisher.publish(forward_clear)
		rate.sleep()
	print("Stopped lidar_controller")

if __name__ == '__main__':
	try:
		print("Starting lidar_controller")
		lidar_data_receiver_setup()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
