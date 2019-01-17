#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import UInt16

# Import and setup for WiringPi (keep order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi

# PWM pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

# Values for the servo
servo_straight = 141
servo_variance = 21.0

# Values for the motor
motor_idle = 150
motor_variance = 14.0
motor_offset = 11

def motor_data_handler(data):
	# higher 8 bit is motor value
	# lower 8 bit is servo value
	received_data = data.data
	motor_received_value = received_data >> 8
	servo_received_value = received_data & 0x00FF

	if (motor_received_value == 50):
		motor_current_value = motor_idle
	elif motor_received_value > 50:
		motor_current_value = (motor_idle + motor_offset) + int( (motor_received_value - 50) / (50.0 / motor_variance) )
	elif motor_received_value < 50:
		motor_current_value = (motor_idle - motor_offset) + int( (motor_received_value - 50) / (50.0 / motor_variance) )
	
	servo_current_value = servo_straight + int( (servo_received_value - 50) / (50.0 / servo_variance) )

	wiringpi.pwmWrite(PWM_PIN_SERVO, servo_current_value)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current_value)

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

def motor_data_receiver_setup():
	rospy.init_node('motor_controller_v2', anonymous=True)
	rospy.Subscriber('lucy/motor_control', UInt16, motor_data_handler)
	# rospy.Subscriber('ultrasonic_sensor_data', Float32, ultrasonic_sensor_data_handler)

if __name__ == '__main__':
	try:
		print("Starting motor_controller_v2")
		motor_data_receiver_setup()
		pwm_setup()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
