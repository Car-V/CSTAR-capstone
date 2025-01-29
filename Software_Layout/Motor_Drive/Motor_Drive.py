from Motor import Motor
from time import sleep
class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(25, 11, 13, 6, 7) # motor 2, in1 in2 enb ch1 ch2
        self.motorR = Motor(10, 9, 12, 5, 8) # motor 1, in1 in2 enb ch1 ch2
        self.bot_idle()

    def bot_idle(self):
        print("we are decelerating")
        while (self.motorL.duty_cycle > 0 or self.motorR.duty_cycle > 0): # clean this up
            self.motorL.decelerate()
            self.motorR.decelerate()
        self.motorL.idle()
        self.motorR.idle()
        print("we are idle")
    
    def bot_forward(self):
        # really need to keep track that the duty cycles for the motors are the same/return to being the same when they should
        self.motorL.forward()
        self.motorR.forward()
        print("we are going forward")
        print("we are accelerating")
        while (self.motorL.duty_cycle < self.motorL.MAX_DUTY_CYCLE or self.motorR.duty_cycle < self.motorR.MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()

    # go idle before going backward
    def bot_backward(self):
        self.motorL.backward()
        self.motorR.backward()
        print("we are going backward")
        print("we are accelerating")
        while (self.motorL.duty_cycle < self.motorL.MAX_DUTY_CYCLE or self.motorR.duty_cycle < self.motorR.MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()

    # go idle before making a turn
    def bot_clockwise(self):
        self.motorL.forward()
        self.motorR.backward()
        print("we are turning clockwise")
        print("we are accelerating")
        while (self.motorL.duty_cycle < self.motorL.MAX_DUTY_CYCLE or self.motorR.duty_cycle < self.motorR.MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()

    # go idle before making a turn
    def bot_counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()
        print("we are turning counter-clockwise")
        print("we are accelerating")
        while (self.motorL.duty_cycle < self.motorL.MAX_DUTY_CYCLE or self.motorR.duty_cycle < self.motorR.MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()

    def delete(self):
        self.motorL.delete()
        print("deleted!")
    
    