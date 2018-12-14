import RPi.GPIO as GPIO
import time



US_TRIGGER = 23
US_ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(US_TRIGGER, GPIO.OUT)
GPIO.setup(US_ECHO, GPIO.IN)

GPIO.output(US_TRIGGER, False)
time.sleep(2)

count = 0
while count < 1000:
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
	print("Distance: ", distance)

	time.sleep(0.1)
	
	count+= 1

GPIO.cleanup()
