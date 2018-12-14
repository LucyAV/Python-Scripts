import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi
import time

US_TRIGGER = 23
US_ECHO = 24

wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(US_TRIGGER, wiringpi.OUTPUT)
wiringpi.pinMode(US_TRIGGER, wiringpi.INPUT)

wiringpi.digitalWrite(US_TRIGGER, 0)
time.sleep(2)

count = 0
while count < 50:
	wiringpi.digitalWrite(US_TRIGGER, 1)
	time.sleep(0.00001)
	wiringpi.digitalWrite(US_TRIGGER, 0)

	while wiringpi.digitalRead(US_ECHO) == 0:
		pulse_start = time.time()

	while wiringpi.digitalRead(US_ECHO) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start
	distance = 17150 * pulse_duration
	distance = round(distance, 2)
	print("Distance: ", distance)

	time.sleep(0.1)
	
	count+= 1

GPIO.cleanup()
