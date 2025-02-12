# Motor.py - controls individual motors.
# This file contains the Motor class, which is used to control individual motors.   


import RPi.GPIO as GPIO          
from time import sleep
from Encoder import Encoder
from PID import PID

class Motor:
    PWM_HZ = 1000
    MAX_DUTY_CYCLE = 25  # May have adjust vals according to our motor
    MIN_DUTY_CYCLE = 10  # Prevent motor stall
    duty_cycle = MAX_DUTY_CYCLE

    def __init__(self, pin1_num: int, pin2_num: int, en_num: int, pin_a: int, pin_b: int):
        self.pin1 = pin1_num
        self.pin2 = pin2_num
        self.enable = en_num
        self.encoder = Encoder(pin_a, pin_b)
        self.pid = PID(Kp, Ki, Kd)  # Initialize PID controller

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)

        self.p = GPIO.PWM(self.enable, Motor.PWM_HZ)
        self.p.start(Motor.duty_cycle)

    def set_target_speed(self, speed):
        self.pid.set_target(speed)  # Set PID target speed
        
    def update_speed(self):
        actual_speed = self.encoder.get_position()  # Read encoder position
        new _duty_cycle = self.pid.compute(actual_speed)  # Compute PID output
        
        # Constrain duty cycle
        new_duty_cycle = max(self.MIN_DUTY_CYCLE, min(self.MAX_DUTY_CYCLE, new_duty_cycle))
        self.p.ChangeDutyCycle(new_duty_cycle)

    def forward(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)
        #self.encoder.update_position()

    def backward(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        #self.encoder.update_position()

    def idle(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        self.p.ChangeDutyCycle(0)

    #def set_speed(self):
        #self.p.ChangeDutyCycle(Motor.duty_cycle)

    # ensure the increments we decelerate or accelerate by are a whole factor of max duty cycle
    @staticmethod
    def accelerate():
        Motor.duty_cycle = Motor.duty_cycle + 5

    @staticmethod
    def decelerate():
        Motor.duty_cycle = Motor.duty_cycle - 5

    def get_encoder_position(self):
        return self.encoder.get_position()

    def get_encoder_direction(self):
        return self.encoder.get_direction()

    def reset_encoder(self):
        self.encoder.reset_position()

    def delete(self):
        GPIO.cleanup()
        print("motor deleted")

