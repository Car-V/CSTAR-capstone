import RPi.GPIO as GPIO          
from time import sleep
from Encoder import Encoder
class Motor:
    def __init__(self, pin1_num: int, pin2_num: int, en_num: int, pin_a: int, pin_b: int):
        self.pin1 = pin1_num
        self.pin2 = pin2_num
        self.enable =  en_num
        self.PWMhz = 1000
        self.duty_cycle = 25
        self.encoder = Encoder(pin_a, pin_b)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)

        p = GPIO.PWM(self.enable, self.PWMhz)
        p.start(self.duty_cycle)


    def forward(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)
        self.encoder.update_position()

    def backward(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        self.encoder.update_position()

    def idle(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)

    def delete(self):
        GPIO.cleanup()

