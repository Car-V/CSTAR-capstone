from Motor import Motor
from time import sleep
class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(25, 11, 13, 6, 7) # motor 2, in1 in2 enb ch1 ch2
        self.motorR = Motor(10, 9, 12, 5, 8) # motor 1, in1 in2 enb ch1 ch2
        self.bot_idle()

    def bot_idle(self):
        self.motorL.idle()
        self.motorR.idle()
        print("we are idle")
    
    def bot_forward(self):
        self.motorL.forward()
        self.motorR.forward()
        print("we are going forward")

    def bot_backward(self):
        self.motorL.backward()
        self.motorR.backward()
        print("we are going backward")

    def bot_clockwise(self):
        self.motorL.forward()
        self.motorR.backward()
        print("we are turning clockwise")

    def bot_counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()
        print("we are turning counter-clockwise")

    def delete(self):
        self.motorL.delete()
        print("deleted!")
    
    