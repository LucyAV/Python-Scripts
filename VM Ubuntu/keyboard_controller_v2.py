#!/usr/bin/env python

# Imports for ROS and the keyboard module
import rospy
from std_msgs.msg import UInt16
import keyboard

# Values for the servo
servo_straight = 50
servo_left = 0
servo_right = 100

# Values for the motor
motor_brake = 0
motor_reverse = 20
motor_idle = 50
motor_normal = 72
motor_boost = 80

# Initialize servo and motor current values
servo_current_value = servo_straight
motor_current_value = motor_idle

# Current states of the keys
is_forward = False
is_reverse = False
is_left = False
is_right = False
is_brake = False
is_boost = False

def key_change_handler(event):
	global servo_current_value
	global motor_current_value	
	global is_forward
	global is_reverse
	global is_left
	global is_right
	global is_brake
	global is_boost

	# 'space' is scan_code 57

	# Detect boost
	if event.scan_code is 42: # 'shift'
		if event.event_type is keyboard.KEY_DOWN:
			is_boost = True
		else:
			is_boost = False

	# Detect motor
	if event.scan_code is 17: # 'w'
		if event.event_type is keyboard.KEY_DOWN:
			is_forward = True
		else:
			is_forward = False
	elif event.scan_code is 31: # 's'
		if event.event_type is keyboard.KEY_DOWN:
			is_brake = True
		else:
			is_brake = False
	elif event.scan_code is 45: # 'x'
		if event.event_type is keyboard.KEY_DOWN:
			is_reverse = True
		else:
			is_reverse = False

	# Apply motor (and boost)
	if is_forward:
		if is_boost:
			motor_current_value = motor_boost
		else:
			motor_current_value = motor_normal
	elif is_reverse:
		motor_current_value = motor_reverse
	elif is_brake:
		motor_current_value = motor_brake
	else:
		motor_current_value = motor_idle

	# Detect servo
	if event.scan_code is 30: # 'a'
		if event.event_type is keyboard.KEY_DOWN:
			is_left = True
		else:
			is_left = False
	elif event.scan_code is 32: # 'd'
		if event.event_type is keyboard.KEY_DOWN:
			is_right = True
		else:
			is_right = False

	# Apply servo
	if is_left:
		servo_current_value = servo_left
	elif is_right:
		servo_current_value = servo_right
	else:
		servo_current_value = servo_straight

def publish_data():
	publisher = rospy.Publisher('lucy/motor_control', UInt16, queue_size=10)
	rospy.init_node('keyboard_controller', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		publisher.publish( (motor_current_value << 8) | servo_current_value )
		rate.sleep()

if __name__ == "__main__":
	keyboard.hook(key_change_handler)

	try:
		publish_data()
	except rospy.ROSInterruptException:
		pass
