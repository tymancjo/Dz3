import datetime
import random

class dz3:
  def __init__(self, name, age):
    self.name = name
    self.age = age

  def myfunc(self):
    print("Hello my name is " + self.name)

class PID:
    """
    this is class created for the PiD controller
    """
    def __init__(self, Kp=1, Ki=0, Kd=0, target = 0, use_time = False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_error = 0
        self.sum_error = 0
        self.use_time = use_time
        self.last_current = 0

        self.last_time = None
        self.target = target

    def update(self, current):
        """
        this function calculates the PiD output
        for given target and current value
        """

        # handling the delta T if needed
        if self.use_time:
            if self.last_time is None:
                # it's the first time 
                self.last_time = datetime.datetime.now()
                self.dt = 1
            else:
                self.dt = datetime.datetime.now() - self.last_time
        else:
            self.dt = 1

        # getting current error
        error = self.target - current
        # adding it to the integral
        self.sum_error += error * self.dt
        #calculating error difference
        error_diff = (error - self.last_error) / self.dt
        # memorizing error
        self.last_error = error
        # memorizing last entry
        self.last_current = current

        # the main PiD calculations:
        PID = self.Kp * error + self.Ki * self.sum_error + self.Kd * error_diff

        return PID  

    def get_error(self, current):
        # getting current error
        error = self.target - current

        return error



    def set_params(self, Kp=1, Ki=0, Kd=0, use_time = False):
        """
        Method to change the internal parameters
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.use_time = use_time


# # testing the created objects classes
# Turn_PID = PID(1,1,1)

# for i in range(20):
#     x = random.random()
#     print(f'{x}: {Turn_PID.update(0.5, x)}')