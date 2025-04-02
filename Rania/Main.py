# Main.py - Motor control with encoder-based odometry
import RPi.GPIO as GPIO
from Encoder import Encoder
from Audio_Collector import Audio_Collector
from Audio_Manager import AudioManager
from Audio_Delam_Testing import Audio_Delam_Testing
# from PID import PID
from time import sleep
import threading
import time
import math

GPIO.setwarnings(False)

# Right Motor
in1 = 17  # PIN 11
in2 = 27  # PIN 13
en_a = 12  # PIN 32 (PWM0)

# Left Motor
in3 = 8  # PIN 24
in4 = 7  # PIN 26
en_b = 13  # PIN 33 (PWM1)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en_a, GPIO.OUT)

GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en_b, GPIO.OUT)

# PWM Setup
q = GPIO.PWM(en_a, 100)  # Right motor PWM
p = GPIO.PWM(en_b, 100)  # Left motor PWM
p.start(15)
q.start(25)

# Stop motors initially
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)

# Encoder Initialization
left_encoder = Encoder(pin_a=16, pin_b=5)   # Left encoder pins
#right_encoder = Encoder(pin_a=2, pin_b=3)    # Right encoder pins

audioCollector = Audio_Collector()
audioTester = Audio_Delam_Testing()


# Start audio recording in a separate thread
def record_audio(duration=5):
    def collect_and_send():
        fft, freq = audioCollector.collect_samples(duration)  # Collect audio data
        audioTester.new_sample(fft, freq)  # Send FFT data to Audio_Delam_Testing

    audio_thread = threading.Thread(target=collect_and_send, daemon=True)
    audio_thread.start()


# def print_odometry():
#     while True:
#         left_encoder.get_rotation_direction()
#         time.sleep(0.1)  

# odometry_thread = threading.Thread(target=print_odometry, daemon=True)
# odometry_thread.start()

# Motor Control Functions
def move_forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    print("Forward")

def move_backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    print("Backward")

def turn_right():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    print("Right")

def turn_left():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    print("Left")

def stop_motors():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    print("Stop")

# Main Loop to Listen for Keyboard Input
try:
    print("Use W/A/S/D to move, C to stop, and CTRL-C to exit.")
    while True:
        # Get user input
        user_input = input().lower()

        if user_input == 'w':
            move_forward()
        elif user_input == 's':
            move_backward()
        elif user_input == 'd':
            turn_right()
        elif user_input == 'a':
            turn_left()
        elif user_input == 'c':
            stop_motors()

# If CTRL-C is pressed, clean up GPIO
except KeyboardInterrupt:
    print("\nKeyboard Interrupt. Stopping and cleaning up...")
finally:
    stop_motors()
    GPIO.cleanup()
    print("GPIO Clean up complete.")
