# Encoder.py - handles encoder readings.
# This file contains the Encoder class, which is used to read the position and direction of a rotary encoder.


import math
import time
import RPi.GPIO as GPIO

class Encoder:
    WHEEL_DIAMETER = 0.072  
    PULSES_PER_REV = 537.7  
    GEAR_RATIO = 19.2       
    WHEEL_BASE = 0.25 

    def __init__(self, pin_a: int, pin_b: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.direction = 0  # 1 for clockwise, -1 for counterclockwise
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        
        self.last_a_state = GPIO.input(self.pin_a)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update_position(self):
        # Read the state of the encoder channel A
        current_a_state = GPIO.input(self.pin_a)
        current_b_state = GPIO.input(self.pin_b)
        # Check if the A channel changed
        if current_a_state != self.last_a_state:
            # If the A channel leads the B channel, it's a clockwise rotation
            if current_b_state != current_a_state:
                self.position += 1
                self.direction = 1  # Clockwise
            else:
                self.position -= 1
                self.direction = -1  # Counterclockwise

        self.last_a_state = current_a_state  # Update the last state for next comparison

    def get_position(self):
        return self.position

    def get_direction(self):
        return self.direction

    def reset_position(self):
        self.position = 0

    
    def calculate_distance(self):
        wheel_circumference = math.pi * self.WHEEL_DIAMETER
        distance_per_pulse = wheel_circumference / (self.PULSES_PER_REV * self.GEAR_RATIO)
        return self.position * distance_per_pulse


    def update_odometry(self, left_encoder, right_encoder):
        left_distance = left_encoder.calculate_distance()
        right_distance = right_encoder.calculate_distance()
        
        distance = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.WHEEL_BASE
        
        print(distance)
        
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

    def get_odometry(self):
        return self.x, self.y, self.theta