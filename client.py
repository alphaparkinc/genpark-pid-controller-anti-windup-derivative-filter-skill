"""Closed-Loop PID Controller with Anti-Windup and Derivative Filter.
100% Python Standard Library.
"""

class PIDController:
    """Industrial PID controller with anti-windup clamping and low-pass filtered derivative."""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-100.0, 100.0), filter_alpha=0.1):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min_out, self.max_out = output_limits
        self.filter_alpha = float(filter_alpha)
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def compute(self, setpoint, measured_value, dt):
        """Compute PID output given current setpoint, feedback measurement, and dt."""
        if dt <= 0.0:
            dt = 1e-4
            
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Derivative term with low-pass filter
        raw_derivative = (error - self.prev_error) / dt
        filtered_derivative = self.filter_alpha * raw_derivative + (1.0 - self.filter_alpha) * self.prev_derivative
        d_term = self.kd * filtered_derivative
        
        # Tentative output calculation
        tentative_out = p_term + self.ki * (self.integral + error * dt) + d_term
        
        # Anti-windup conditional integration
        if tentative_out > self.max_out:
            output = self.max_out
            if error < 0.0:
                self.integral += error * dt
        elif tentative_out < self.min_out:
            output = self.min_out
            if error > 0.0:
                self.integral += error * dt
        else:
            self.integral += error * dt
            output = tentative_out
            
        self.prev_error = error
        self.prev_derivative = filtered_derivative
        return output
