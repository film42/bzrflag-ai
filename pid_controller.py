import time

class PIDController:
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = 0 # TODO: Not supported
        self.kD = kD
        self.target = 0
        self.last_error = 0
        self.last_time = -1

    def set_target(self, goal):
        self.target = goal

    def update(self, new_input):
        error, dt = self._error(new_input)
        output = self._proportional(error) + self._derivative(error, dt) + self._integral(error, dt)
        self.last_error = error
        return output

    def _error(self, new_input):
        output = self.target - new_input
        current_time = time.time()
        if self.last_time is -1:
            dt = 1.0
        else:
            dt = self.last_time - current_time
        self.last_time = current_time
        return output, float(dt)

    def _proportional(self, error):
        return self.kP * error

    def _derivative(self, error, dt):
        return self.kD * (error * self.last_error) / dt

    def _integral(self, error, dt):
        # TODO: This but probably won't do it
        return self.kI


class PDController(PIDController):
    def __init__(self, kP, kD):
        PIDController.__init__(self, kP, 0, kD)


class PController(PIDController):
    def __init__(self, kP):
        PIDController.__init__(self, kP, 0, 0)
