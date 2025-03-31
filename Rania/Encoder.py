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
        
        # Initialize states
        self.last_a_state = GPIO.input(self.pin_a)
        self.last_b_state = GPIO.input(self.pin_b)

        # Add event detection for both pins (rising or falling edges)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.encoder_callback)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self.encoder_callback)

    def encoder_callback(self, channel):
        # This function will be called when either pin A or B changes state
        curr_a = GPIO.input(self.pin_a)
        curr_b = GPIO.input(self.pin_b)
        print("Callback Triggered: Curr State - A: {}, B: {}".format(curr_a, curr_b))

        # Use the previous state to determine the direction
        if (self.last_a_state, self.last_b_state) == (0, 0) and (curr_a, curr_b) == (1, 0):
            self.direction = 1  # Clockwise (00 -> 10)
        elif (self.last_a_state, self.last_b_state) == (1, 0) and (curr_a, curr_b) == (1, 1):
            self.direction = 1  # Clockwise (10 -> 11)
        elif (self.last_a_state, self.last_b_state) == (1, 1) and (curr_a, curr_b) == (0, 1):
            self.direction = 1  # Clockwise (11 -> 01)
        elif (self.last_a_state, self.last_b_state) == (0, 1) and (curr_a, curr_b) == (0, 0):
            self.direction = 1  # Clockwise (01 -> 00)
        elif (self.last_a_state, self.last_b_state) == (0, 0) and (curr_a, curr_b) == (0, 1):
            self.direction = -1  # Counterclockwise (00 -> 01)
        elif (self.last_a_state, self.last_b_state) == (0, 1) and (curr_a, curr_b) == (1, 1):
            self.direction = -1  # Counterclockwise (01 -> 11)
        elif (self.last_a_state, self.last_b_state) == (1, 1) and (curr_a, curr_b) == (1, 0):
            self.direction = -1  # Counterclockwise (11 -> 10)
        elif (self.last_a_state, self.last_b_state) == (1, 0) and (curr_a, curr_b) == (0, 0):
            self.direction = -1  # Counterclockwise (10 -> 00)

        # Update last states for next callback
        self.last_a_state = curr_a
        self.last_b_state = curr_b

        # Optionally print the direction when the callback triggers
        print("Direction: ", self.direction)

    def get_rotation_direction(self):
        # This method can still be used to return the current direction if needed
        return self.direction

    def cleanup(self):
        # Disable event detection when done
        GPIO.remove_event_detect(self.pin_a)
        GPIO.remove_event_detect(self.pin_b)
