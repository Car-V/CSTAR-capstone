from Motor import Motor
class Motor_Drive:
    def __init__(self):
        self.motorL = Motor(24, 23, 25)
        self.motorR = Motor(5, 6, 4)
        self.idle()

    def idle(self):
        self.motorL.idle()
        self.motorR.idle()
    
    def forward(self):
        self.motorL.forward()
        self.motorR.forward()

    def backward(self):
        self.motorL.backward()
        self.motorR.backward()

    def clockwise(self):
        self.motorL.forward()
        self.motorR.backward()

    def counter_clockwise(self):
        self.motorL.backward()
        self.motorR.forward()

    def delete(self):
        self.motorL.delete()
        self.motorR.delete()
    
    