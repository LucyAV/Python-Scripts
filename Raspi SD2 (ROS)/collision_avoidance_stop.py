#!/usr/bin/env python

# Imports for ROS and the keyboard module
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool

# Values for the servo
servo_straight = 50
servo_left = 0
servo_right = 100

# Values for the motor
motor_brake = 0
motor_reverse = 28
motor_idle = 50
motor_normal = 51
motor_boost = 100

# Initialize servo and motor current values
servo_current_value = servo_straight
motor_current_value = motor_idle

# State of driving and forward clearance
isDriving = False
lidar_forward_clear = True

# Global publisher
publisher = None

def drive_until_obstacle():
	global isDriving
	global motor_current_value
	global servo_current_value

	print("DRIVE: Start")
	servo_current_value = servo_straight

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		if lidar_forward_clear is True:
			motor_current_value = 51
			#isDriving = True
			publisher.publish( (motor_current_value << 8) | servo_current_value )
			rate.sleep()
		else:
			print("DRIVE: Stop")
			break
	
	motor_current_value = motor_brake
	publisher.publish( (motor_current_value << 8) | servo_current_value )
	rospy.sleep(1.5)
	motor_current_value = motor_idle
	publisher.publish( (motor_current_value << 8) | servo_current_value )

def lidar_forward_clear_handler(data):
	global lidar_forward_clear
	lidar_forward_clear = data.data

def publish_data():
	global publisher
	publisher = rospy.Publisher('lucy/motor_control', UInt16, queue_size=10)
	rospy.init_node('collision_avoidance_stop', anonymous=True)
	rospy.Subscriber('/lucy/lidar_forward_clear', Bool, lidar_forward_clear_handler)

	drive_until_obstacle()

if __name__ == "__main__":
	try:
		print("Starting collision_avoidance_stop")
		publish_data()
		
	except rospy.ROSInterruptException:
		pass
