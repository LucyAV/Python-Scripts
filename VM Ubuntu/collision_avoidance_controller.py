#!/usr/bin/env python

# Imports for ROS and the keyboard module
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
import keyboard

# Values for the servo
servo_straight = 50
servo_left = 0
servo_right = 100

# Values for the motor
motor_brake = 0
motor_reverse = 28
motor_idle = 50
motor_normal = 72
motor_boost = 100

# Initialize servo and motor current values
servo_current_value = servo_straight
motor_current_value = motor_idle

# Current states of the keys
isDriving = False

lidar_forward_clear = False

publisher = None

def drive_until_obstacle():
	global isDriving
	global motor_current_value
	global servo_current_value

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if lidar_forward_clear is True:
			servo_current_value = servo_left
			isDriving = True
		else:
			servo_current_value = servo_straight
			isDriving = False
			print("Stopped")

		publisher.publish( (motor_current_value << 8) | servo_current_value )

		if (isDriving is False):
			break
		else:
			rate.sleep()

def key_change_handler(event):
	# 'space' is scan_code 57

	# Detect autonomous mode ('l')
	if (event.scan_code is 38) and (event.event_type is keyboard.KEY_UP) and (isDriving is False):
		drive_until_obstacle()

def lidar_forward_clear_handler(data):
	global lidar_forward_clear
	lidar_forward_clear = data.data

def publish_data():
	global publisher
	publisher = rospy.Publisher('lucy/motor_control', UInt16, queue_size=10)
	rospy.init_node('keyboard_controller', anonymous=True)
	rospy.Subscriber('/lucy/lidar_forward_clear', Bool, lidar_forward_clear_handler)

	rospy.spin()

if __name__ == "__main__":
	keyboard.hook(key_change_handler)

	try:
		publish_data()
	except rospy.ROSInterruptException:
		pass
