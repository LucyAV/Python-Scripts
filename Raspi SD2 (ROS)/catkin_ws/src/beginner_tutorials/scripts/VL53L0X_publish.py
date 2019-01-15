#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import UInt16

# Imports for rounding numbers
import math

# module
import sys
sys.path.append('/usr/local/lib/python2.7/dist-packages/VL53L0X_rasp_python/python/VL53L0X.py')

# Imports for tof measurements
import time
import VL53L0X

def measure_and_publish_data():
	publisher = rospy.Publisher('tof_sensor_data', UInt16, queue_size=10)
	rospy.init_node('tof_sensor_controller', anonymous=True)

	# Create a VL53L0X object
	tof = VL53L0X.VL53L0X()

	# Start ranging
	tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

	timing = tof.get_timing()
	if (timing < 20000):
	    timing = 20000
	print ("Timing %d ms" % (timing/1000))
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		distance = tof.get_distance()

		publisher.publish(distance)
		rate.sleep()
	tof.stop_ranging()
	print("Stopped tof_sensor_controller")

if __name__ == "__main__":
	try:
		print("Starting tof_sensor_controller")
		#setup_ultrasonic_sensor()
		measure_and_publish_data()
	except rospy.ROSInterruptException:
		pass