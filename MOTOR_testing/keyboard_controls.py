# Purpose: Enables manual control of two DC motors using keyboard inputs (forward, backward, left, right, and stop).

import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)

# Right Motor

# PIN 11
in1 = 17

# PIN 13
in2 = 27

#Switch to pwm gpio 
# PIN 7 (GPCLK0)
#en_a = 4

# PIN 32 (PWM0)
en_a = 12 


# Left Motor

# PIN 29 (GPCLK1)
# Dont have to be connected to clk; can be any gpio
# in3 = 5

# PIN 24 
in3 = 8

# PIN 31 (GPCLK2)
# dont have to be connected to clk; can be any gpio
# in4 = 6

# PIN 26
in4 = 7


# PIN 33 (PWM1)
en_b = 13 


GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en_b,GPIO.OUT)

#q=GPIO.PWM(en_a,100)
#p=GPIO.PWM(en_b,100)
#p.start(75)
#q.start(75)


# Adjust these values to fine-tune straight movement
LEFT_MOTOR_SPEED = 65  # Slightly slower, adjust down if still veering right
RIGHT_MOTOR_SPEED = 75  # Reference speed

q = GPIO.PWM(en_a, 100)  # Right motor PWM
p = GPIO.PWM(en_b, 100)  # Left motor PWM

p.start(LEFT_MOTOR_SPEED)
q.start(RIGHT_MOTOR_SPEED)



GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)

# Wrap main content in a try block so we can  catch the user pressing CTRL-C and run the
# GPIO cleanup function. This will also prevent the user seeing lots of unnecessary error messages.
try:
# Create Infinite loop to read user input
   while(True):
      # Get user Input
      user_input = input()

      # To see users input
      # print(user_input)

      if user_input == 'w':
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)

         GPIO.output(in4,GPIO.HIGH)
         GPIO.output(in3,GPIO.LOW)

         print("Forward")

      elif user_input == 's':
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)

         GPIO.output(in4,GPIO.LOW)
         GPIO.output(in3,GPIO.HIGH)
         print('Back')

      elif user_input == 'd':
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)

         GPIO.output(in4,GPIO.LOW)
         GPIO.output(in3,GPIO.LOW)
         print('Right')

      elif user_input == 'a':
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)

         GPIO.output(in4,GPIO.LOW)
         GPIO.output(in3,GPIO.LOW)
         print('Left')

      # Press 'c' to exit the script
      elif user_input == 'c':
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.LOW)

         GPIO.output(in4,GPIO.LOW)
         GPIO.output(in3,GPIO.LOW)
         print('Stop')

# If user press CTRL-C
except KeyboardInterrupt:
  # Reset GPIO settings
  GPIO.cleanup()
  print("GPIO Clean up")