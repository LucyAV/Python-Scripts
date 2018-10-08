import socket
import wiringpi
from time import sleep
import threading
import time
import RPi.GPIO as GPIO

# WiringPi servo and motor pins
SERVO_PIN = 18
MOTOR_PIN = 19
# Motor stopping, braking, current and received duty cycle
MOTOR_STOP = 150
MOTOR_BRAKE = 130
motorSpeedCurrent = MOTOR_STOP
motorSpeedReceived = MOTOR_STOP

# RPi ultrasonic sensor pins
SENSOR_1_TRIGGER_PIN = 23
SENSOR_1_ECHO_PIN = 24
# Last three measured distance values from ultrasonic sensor 1
sensor1Distance1 = 1
sensor1Distance2 = 1
sensor1Distance3 = 1


# ------------------------------------------------------------ #
# Setup of the two PWM channels #

# WiringPi PWM variables
PWM_CLOCK = 192;
PWM_RANGE = 2000;

# Initialize WiringPi, then pin, then PWM mode
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(SERVO_PIN, 2)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
# Configure frequency
wiringpi.pwmSetClock(PWM_CLOCK)
wiringpi.pwmSetRange(PWM_RANGE)
# Set duty cycle (straight for servo is 140)
wiringpi.pwmWrite(SERVO_PIN, 140)

# Initialize WiringPi, then pin, then PWM mode
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(MOTOR_PIN, 2)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
# Configure frequency
wiringpi.pwmSetClock(PWM_CLOCK)
wiringpi.pwmSetRange(PWM_RANGE)
# Set duty cycle (stop for motor 150)
wiringpi.pwmWrite(MOTOR_PIN, MOTOR_STOP)


# ------------------------------------------------------------ #
# Ultrasonic sensors #

# Thread for ultrasonic sensor
class ultrasonicSensorThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        # Set up pins for ultrasonic sensor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SENSOR_1_TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(SENSOR_1_ECHO_PIN, GPIO.IN)
        GPIO.output(SENSOR_1_TRIGGER_PIN, False)
        sleep(0.1)
    def run(self):
        # Define variables
        global sensor1Distance1
        global sensor1Distance2
        global sensor1Distance3
        lastDistance = 0
        secondLastDistance = 0
        
        thresholdDistance = 150
        lastMotorSpeed = 150

        # Continuously measure distance in front of the vehicle with
        #  ultrasonic sensor and decide on motorSpeed based on the measurement
        while True:
            # Trigger ultrasonic sensor
            GPIO.output(SENSOR_1_TRIGGER_PIN, True)
            sleep(0.00001)
            GPIO.output(SENSOR_1_TRIGGER_PIN, False)

            # Store time of trigger
            while GPIO.input(SENSOR_1_ECHO_PIN) == 0:
                pulse_start = time.time()

            # Store time of return
            while GPIO.input(SENSOR_1_ECHO_PIN) == 1:
                pulse_end = time.time()

            # Calculate distance from time delta
            pulse_duration = pulse_end - pulse_start
            distance = 17100 * pulse_duration # 17100 better than 17150
            #distance = round(distance, 2)

            # Refresh the last three measured distance values
            sensor1Distance3 = sensor1Distance2
            sensor1Distance2 = sensor1Distance1
            sensor1Distance1 = int(distance)

            # Sensor timing (10 times per second)
            sleep(0.1)


# ------------------------------------------------------------ #
# Motor control #

# Determines if the path in front of the vehicle is clear to drive for a
# given speed
def clearPath(thresholdDistance):
    if (sensor1Distance1 > thresholdDistance):
        if (sensor1Distance2 > thresholdDistance):
            if (sensor1Distance3 > thresholdDistance):
                return True
    return False

# function for braking
def brake():
    print("Initiating emergency brake.")
    wiringpi.pwmWrite(MOTOR_PIN, MOTOR_BRAKE)
    sleep(0.6)
    wiringpi.pwmWrite(MOTOR_PIN, MOTOR_STOP)
    print("Vehicle should have stopped.")

# Thread for motor control
class motorControlThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        while True:
            if 126 <= motorSpeedReceived <= 150:
                motorSpeedCurrent = motorSpeedReceived
            elif 151 <= motorSpeedReceived <= 160:
                if clearPath(50):
                    motorSpeedCurrent = motorSpeedReceived
                else:
                    motorSpeedCurrent = MOTOR_STOP
            elif 161 <= motorSpeedReceived <= 165:
                if clearPath(100):
                    motorSpeedCurrent = motorSpeedReceived
                else:
                    motorSpeedCurrent = MOTOR_STOP
            elif 166 <= motorSpeedReceived <= 170:
                if clearPath(150):
                    motorSpeedCurrent = motorSpeedReceived
                else:
                    motorSpeedCurrent = MOTOR_STOP
            elif 171 <= motorSpeedReceived <= 174:
                if clearPath(200):
                    motorSpeedCurrent = motorSpeedReceived
                else:
                    motorSpeedCurrent = MOTOR_STOP
            wiringpi.pwmWrite(MOTOR_PIN, motorSpeedCurrent)

            # Motor timing (100 times per second)
            sleep(0.01)


# ------------------------------------------------------------ #
# Creation and start of the additional threads #

# Create and start the ultrasonic sensor thread
ust = ultrasonicSensorThread()
ust.daemon = True
ust.start()
# Create and start the motor control thread
mct = motorControlThread()
mct.daemon = True
mct.start()


# ------------------------------------------------------------ #
# Network connection and data receiving #

# Socket variables
HOST = "10.0.0.232"
#HOST = "192.168.43.232"
PORT = 23232

# Create socket and wait for connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
print("Waiting for connection...")
conn, addr = s.accept()
print("Connected to ", addr)
while True:
    # Listen for data
    data = conn.recv(1024)
    
    if data:
        # If data is received, check if there was a delay in the transmission
        values = data.decode().split(" ");
        if len(values) > 2:
            print("Delay...")
            print(values)
            # BRAKE HERE?
        else:
            # Set steering angle based on newly received data
            wiringpi.pwmWrite(SERVO_PIN, int(values[0]))

            # Store the received motor speed
            motorSpeedReceived = int(values[1])
    else:
        print("Connection lost. Resetting motor, servo and ultrasonic sensor.")
        
        wiringpi.pwmWrite(SERVO_PIN, 140)
        wiringpi.pwmWrite(MOTOR_PIN, MOTOR_STOP)
        sleep(0.25)
        
        wiringpi.pinMode(SERVO_PIN, 1)
        wiringpi.digitalWrite(18, 0)
        wiringpi.pinMode(SERVO_PIN, 0)

        wiringpi.pinMode(MOTOR_PIN, 1)
        wiringpi.digitalWrite(19, 0)
        wiringpi.pinMode(MOTOR_PIN, 0)

        GPIO.cleanup()
        
        break;
