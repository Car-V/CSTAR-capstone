from Motor import Motor
class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(24, 23, 25)
        self.motorR = Motor(5, 6, 4)
        self.bot_idle()

    def bot_idle(self):
        self.motorL.idle()
        self.motorR.idle()
    
    def bot_forward(self):
        self.motorL.forward()
        self.motorR.forward()

    def bot_backward(self):
        self.motorL.backward()
        self.motorR.backward()

    def bot_clockwise(self):
        self.motorL.forward()
        self.motorR.backward()

    def bot_counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()

    def delete(self):
        self.motorL.delete()
        self.motorR.delete()
    
    