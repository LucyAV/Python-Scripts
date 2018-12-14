#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import Float32

# Imports for ultrasonic measurements
import RPi.GPIO as GPIO
import time

# Ultrasonic sensor pins
US_TRIGGER = 23
US_ECHO = 24

def setup_ultrasonic_sensor():
	# Pin setup
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(US_TRIGGER, GPIO.OUT)
	GPIO.setup(US_ECHO, GPIO.IN)

	GPIO.output(US_TRIGGER, False)
	time.sleep(1)

def measure_and_publish_data():
	publisher = rospy.Publisher('ultrasonic_sensor_data', Float32, queue_size=10)
	rospy.init_node('ultrasonic_sensor_controller', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		GPIO.output(US_TRIGGER, True)
		time.sleep(0.00001)
		GPIO.output(US_TRIGGER, False)

		while GPIO.input(US_ECHO) == 0:
			pulse_start = time.time()

		while GPIO.input(US_ECHO) == 1:
			pulse_end = time.time()

		pulse_duration = pulse_end - pulse_start
		distance = 17150 * pulse_duration
		distance = round(distance, 2)

		publisher.publish(distance)
		rate.sleep()
	print("Stopped ultrasonic_sensor_controller")

if __name__ == "__main__":
	try:
		print("Starting ultrasonic_sensor_controller")
		setup_ultrasonic_sensor()
		measure_and_publish_data()
	except rospy.ROSInterruptException:
		pass