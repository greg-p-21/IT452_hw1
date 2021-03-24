# Gregory Polmatier
# code derived from https://simple-pid.readthedocs.io/en/latest/_modules/simple_pid/PID.html#PID

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value

class PID:
    def __init__(
        self, 
        kp=1.0, 
        ki=0.0, 
        kd=0.0, 
        setpoint=0, 
        output_limits=(None, None),
        num_errors=10
    ):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.num_errors = num_errors

        self.errors = []

        self._proportional = 0
        self._integral = 0
        self._derivative = 0


        self._last_output = None
        self._last_input = None
        
        self.output_limits = output_limits
        self.reset()

    def __call__(self, input_):
        # compute error term
        error = self.setpoint - input_ 

        # compute proportional
        self._proportional = self.kp * error

        # compute integral
        error_sum = sum(self.errors[-self.num_errors:]) if (len(self.errors) > self.num_errors) else sum(self.errors)
        self._integral = self.ki * error_sum
        self._integral = _clamp(self._integral, self.output_limits)

        # compute derivative 
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)
        self._derivative = self.kd * d_input

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)
        
        # keep track 
        self.errors.append(error)
        self._last_output = output
        self._last_input = input_

        return output 

    @property
    def components(self):
        return self._proportional, self._integral, self._derivative

    def reset(self):
        """
        Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_output = None
        self._last_input = None

if __name__ == "__main__":
    pid = PID()
    pid(5)
    print(pid.components)
    pid(3)
    print(pid.components)
    pid(1)
    print(pid.components)
