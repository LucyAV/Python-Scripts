import wiringpi
from time import sleep

# 117 to 162 are possible values for steering servo - 118 to 161 are safer)

numClock = 192;
numRange = 2000;

### Initialize WiringPi, then Pin, then PWM Mode
##wiringpi.wiringPiSetupGpio()
##wiringpi.pinMode(18, 2)
##wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
##
### Configure Frequency
##wiringpi.pwmSetClock(numClock)
##wiringpi.pwmSetRange(numRange)
##
### Set Duty Cycle
##wiringpi.pwmWrite(18, 140)


# Initialize WiringPi, then Pin, then PWM Mode
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(19, 2)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# Configure Frequency
wiringpi.pwmSetClock(numClock)
wiringpi.pwmSetRange(numRange)

# Set Duty Cycle
print("Calibration ready. Turn ESC on within 10 seconds.")
wiringpi.pwmWrite(19, 150)
sleep(14)
print("Calibration should be completed by now.")
