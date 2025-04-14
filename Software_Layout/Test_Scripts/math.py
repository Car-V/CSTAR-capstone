#echo none > /sys/class/leds/led0/trigger

# taken from Google AI

#import RPi.GPIO as GPIO
#from time import sleep

# Set GPIO mode to BCM numbering
#GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for the onboard

value = 0;
for num in range(40):
	if num % 5 == 0:
		value = value + num/5
		print(value)
print("all done!")

