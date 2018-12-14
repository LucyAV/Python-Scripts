#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import Float32, UInt16

# Import and setup for WiringPi (keep order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi

# PWM pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

# Ultrasonic sensor distance
# ultrasonic_sensor_distance = 0

def motor_data_handler(data):
	servo_current_value = data.data / 100
	motor_current_value = (data.data - (servo_current_value * 100)) + 100
	servo_current_value = servo_current_value + 100

	wiringpi.pwmWrite(PWM_PIN_SERVO, servo_current_value)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current_value)

# def ultrasonic_sensor_data_handler(data):
# 	global ultrasonic_sensor_distance
# 	ultrasonic_sensor_distance = data.data

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
	rospy.init_node('motor_controller', anonymous=True)
	rospy.Subscriber('motor_data', UInt16, motor_data_handler)
	# rospy.Subscriber('ultrasonic_sensor_data', Float32, ultrasonic_sensor_data_handler)

if __name__ == '__main__':
	try:
		print("Starting motor_controller")
		data_receiver_setup()
		pwm_setup()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
