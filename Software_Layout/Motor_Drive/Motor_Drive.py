from Motor import Motor
from time import sleep
class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(25, 11, 13, 6, 7) # motor 2, in1 in2 enb ch1 ch2
        self.motorR = Motor(10, 9, 12, 5, 8) # motor 1, in1 in2 enb ch1 ch2
        self.bot_idle()

    def bot_idle(self):
        while (self.motorL.duty_cycle > 0 or self.motorR.duty_cycle > 0):
            self.motorL.decelerate()
            self.motorR.decelerate()
            print("we are decelerating")
        self.motorL.idle()
        self.motorR.idle()
        print("we are idle")
    
    def bot_forward(self):
        # really need to keep track that the duty cycles for the motors are the same/return to being the same when they should
        self.motorL.forward()
        self.motorR.forward()
        while (self.duty_cycle < MAX_DUTY_CYCLE or self.motorR.duty_cycle < MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()
            print("we are accelerating")
        print("we are going forward")

    # go idle before going backward
    def bot_backward(self):
        self.motorL.backward()
        self.motorR.backward()
        while (self.duty_cycle < MAX_DUTY_CYCLE or self.motorR.duty_cycle < MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()
            print("we are accelerating")
        print("we are going backward")

    # go idle before making a turn
    def bot_clockwise(self):
        self.motorL.forward()
        self.motorR.backward()
        while (self.duty_cycle < MAX_DUTY_CYCLE or self.motorR.duty_cycle < MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()
            print("we are accelerating")
        print("we are turning clockwise")

    # go idle before making a turn
    def bot_counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()
        while (self.duty_cycle < MAX_DUTY_CYCLE or self.motorR.duty_cycle < MAX_DUTY_CYCLE):
            self.motorL.accelerate()
            self.motorR.accelerate()
            print("we are accelerating")
        print("we are turning counter-clockwise")

    def delete(self):
        self.motorL.delete()
        print("deleted!")
    
    