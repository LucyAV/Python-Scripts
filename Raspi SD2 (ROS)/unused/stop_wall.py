#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import UInt16

# Import and setup for WiringPi (keep order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi
import time

# PWM pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

# Ultrasonic sensor distance
ultrasonic_sensor_distance = 400

# TOF sensor distance
tof_sensor_distance = 2000

def ultrasonic_sensor_data_handler(data):
	global ultrasonic_sensor_distance
	ultrasonic_sensor_distance = data.data

def tof_sensor_data_handler(data):
	global tof_sensor_distance
	tof_sensor_distance = data.data

def pwm_setup():
	# Clock variables
	PWM_CLOCK = 192;
	PWM_RANGE = 2000;

	# Initialize WiringPi, then Pin, then PWM Mode
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_SERVO, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_SERVO, 141)

	# Initialize WiringPi, then Pin, then PWM Mode
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_MOTOR, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, 150)

def data_receiver_setup():
	rospy.init_node('stop_wall', anonymous=True)
	#rospy.Subscriber('ultrasonic_sensor_data', Float32, ultrasonic_sensor_data_handler)
	rospy.Subscriber('tof_sensor_data', UInt16, tof_sensor_data_handler)

def drive_until_obstacle():
	is_obstacle = False

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if (tof_sensor_distance != 0) and (tof_sensor_distance < 1000):
			is_obstacle = True

		if not is_obstacle:
			wiringpi.pwmWrite(PWM_PIN_MOTOR, 160)
		else:
			break
		rate.sleep()

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 126)
	time.sleep(1)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, 150)
	print("HALT")

	# while not rospy.is_shutdown():
	# 	if ultrasonic_sensor_distance > 200:
	# 		drive_until_obstacle()

if __name__ == '__main__':
	try:
		print("Starting stop_wall")
		data_receiver_setup()
		pwm_setup()
		print("Setup done, starting to drive")
		drive_until_obstacle()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
