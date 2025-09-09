from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks._common import MaybeAwaitable
from pybricks.tools import StopWatch, wait

def warn(message):
    """Print a warning message"""
    print("Warning:", message)
    
class BetterMotor(Motor):
    def __init__(self, port, default_speed, positive_direction=Direction.CLOCKWISE, gears=None, reset_angle=True, profile=None):
        self.default_speed = default_speed
        super().__init__(port, positive_direction, gears, reset_angle, profile)
    
    def run_target(self, speed, angle=None, then=Stop.HOLD, wait=True, duration=None, fallbackWhenNotStalled=None, fallbackWhenStalled=None, diff=5):
        def default_callback():
            pass
            
        if fallbackWhenNotStalled is None:
            fallbackWhenNotStalled = default_callback
        if fallbackWhenStalled is None:
            fallbackWhenStalled = default_callback
            
        if angle is None:
            super().run_target(self.default_speed, speed, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(speed - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        else:
            super().run_target(speed, angle, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(angle - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        self.brake()
        
    def run_angle(self, speed, angle=None, then=Stop.HOLD, wait=True, duration=None, fallbackWhenNotStalled=None, fallbackWhenStalled=None, diff=5):
        def default_callback():
            pass
            
        if fallbackWhenNotStalled is None:
            fallbackWhenNotStalled = default_callback
        if fallbackWhenStalled is None:
            fallbackWhenStalled = default_callback
            
        oldAngle = self.angle()
        if angle is None:
            super().run_angle(self.default_speed, speed, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(oldAngle + speed - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        else:
            super().run_angle(speed, angle, then, wait=False)
            if duration is None:
                if wait:
                    while not self.done():
                        pass
                return
            sw = StopWatch()
            while sw.time() < duration:
                if self.done():
                    fallbackWhenNotStalled()
                    return
            if abs(oldAngle + angle - self.angle()) > diff:
                fallbackWhenStalled()
            else:
                fallbackWhenNotStalled()
        self.brake()

    def set_default_speed(self, new_speed):
        self.default_speed = new_speed

class BetterDriveBase(DriveBase):
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
    
    def drive(self, speed, turn_rate=0):
        super().drive(speed, turn_rate)
    
    def settings(self, straight_speed=None, straight_acceleration=None, turn_rate=None, turn_acceleration=None, checkMinimumSpeed=True):
        if straight_speed is None and straight_acceleration is None and turn_rate is None and turn_acceleration is None:
            return super().settings()
        elif checkMinimumSpeed:
            if straight_speed < 200:
                raise Exception("Speed under 200. ")
            super().settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
        else:
            if straight_speed < 200:
                warn("Accepted speed under 200. ")
            else:
                warn("Function may take speed under 200. Set checkMinimumSpeed to true to throw an error when trying to do that. ")
            super().settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
        wait(50)