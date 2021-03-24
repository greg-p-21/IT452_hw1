# Gregory Polmatier
# code derived from https://simple-pid.readthedocs.io/en/latest/_modules/simple_pid/PID.html#PID

class PID:
    def __init__(self, kp=1, ki=0, kd=0, setpoint=0, num_errors=10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.num_errors = num_errors

        self._prev_errors = []

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_output = None
        self._last_input = None
        
        self.reset()

    def __call__(self, input_):
        # compute error terms
        error = self.setpoint - input_ 

        # compute proportional
        self._proportional = self.kp * error

        # compute integral
        error_sum = (sum(self._prev_errors[-self.num_errors:]) if (len(self._prev_errors) >= self.num_errors) else sum(self._prev_errors))
        self._integral = self.ki * error_sum

        # compute derivative 
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)
        self._derivative = self.kd * d_input

        # compute final output
        output = self._proportional + self._integral + self._derivative
        
        # keep track 
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
