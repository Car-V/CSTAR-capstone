# PID.py - new Python module to handle PID logic.


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Target speed
        self.integral = 0
        self.previous_error = 0

    def compute(self, actual_value):
        error = self.setpoint - actual_value  # Difference between target and actual
        self.integral += error
        derivative = error - self.previous_error
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error
        return max(0, min(100, output))  # Ensure PWM is within 0-100%
    
    def set_target(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0  # Reset integral term
        self.previous_error = 0  # Reset derivative term
