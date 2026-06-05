from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from pybricks.hubs import PrimeHub
try:
    from pybricks import hubs
except ImportError:
    pass
from math import ceil, pi, tan, radians, sin, atan, degrees


def warn(message):
    """Print a warning message"""
    print("Warning:", message)


class BetterMotor(Motor):
    def __init__(self, port, default_speed,
                 positive_direction=Direction.CLOCKWISE,
                 gears=None, reset_angle=True, profile=None):
        self.default_speed = default_speed
        super().__init__(port, positive_direction, gears, reset_angle, profile)

    def run_target(self, speed, angle=None,
                   then=Stop.HOLD, wait=True,
                   duration=None, fallbackWhenNotStalled=None,
                   fallbackWhenStalled=None, diff=5):
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

    def run_angle(self, speed, angle=None, then=Stop.HOLD, wait=True,
                  duration=None, fallbackWhenNotStalled=None,
                  fallbackWhenStalled=None, diff=5):
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

    def settings(self, straight_speed=None, straight_acceleration=None,
                 turn_rate=None, turn_acceleration=None,
                 checkMinimumSpeed=True):
        if (straight_speed is None and straight_acceleration is None and
                turn_rate is None and turn_acceleration is None):
            return super().settings()
        elif checkMinimumSpeed:
            if straight_speed < 200:
                raise Exception("Speed under 200. ")
            super().settings(straight_speed, straight_acceleration,
                             turn_rate, turn_acceleration)
        else:
            if straight_speed < 200:
                warn("Accepted speed under 200. ")
            else:
                warn("Function may take speed under 200. Set checkMinimumSpe" +
                     "ed to true to throw an error when trying to do that. ")
            super().settings(straight_speed, straight_acceleration,
                             turn_rate, turn_acceleration)
        wait(50)

class CarDriveBase:
    def __init__(self, drive_motor, steer_motor, wheel_diameter,
                 axle_track, default_speed, default_steer_speed, hub):
        self.drive_motor = drive_motor  # type: Motor
        self.steer_motor = steer_motor  # type: Motor
        self.wheel_diameter = wheel_diameter  # type: float
        self.axle_track = axle_track  # type: float
        self.default_speed = default_speed  # type: int
        self.default_steer_speed = default_steer_speed  # type: int
        self.hub = hub  # type: PrimeHub
        self.gyro = False
        self.running = False

    def use_gyro(self, use):
        self.hub.imu.reset_heading(0)
        self.gyro = use
        
    def reset_gyro(self):
        if self.gyro:
            self.hub.imu.reset_heading(0)
        
    def correct(self, angle=0, step=30, pr=False):
        if self.gyro:
            if pr:
                print(max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))))
            self.steer_motor.run_target(self.default_steer_speed, max(-abs(step), min(abs(step), 15 * ceil((angle - self.hub.imu.heading()) / 15))), wait=False)

    def drive(self, speed=None):
        if speed is None:
            speed = self.default_speed
        self.drive_motor.run(speed)
        self.running = True

    def stop(self):
        self.drive_motor.stop()
        self.running = False

    def brake(self):
        self.drive_motor.brake()
        self.running = False

    def _straight(self, dist, turn_rate=0, speed=None, _gyro=True):
        self.running = True
        if speed is None:
            speed = self.default_speed
        o = self.wheel_diameter * pi
        degrees = dist / o * 360
        speed = abs(speed) * (1 if degrees > 0 else -1)
        start = self.drive_motor.angle()
        self.drive_motor.run(speed)
        while abs(self.drive_motor.angle() - start) < abs(degrees):
            if self.gyro and _gyro:
                self.correct(turn_rate)
            wait(5)
        self.drive_motor.brake()
        self.running = False

    def straight(self, dist, turn_rate=0, speed=None):
        self.steer_motor.run_target(self.default_steer_speed, 0)
        self._straight(dist, turn_rate, speed)

    def turn(self, target_deg, step_deg, tolerance=1.5, speed_steer=None, speed_drive=None):
        if target_deg == 0:
            return
        if step_deg == 0:
            self.steer_motor.run_target(self.default_steer_speed, 0)
            return
        ta = target_deg / abs(target_deg)
        sa = step_deg / abs(step_deg)
        if speed_steer is None:
            speed_steer = self.default_steer_speed
        if speed_drive is None:
            speed_drive = self.default_speed

        axle_track_mm = self.axle_track * 10  # cm to mm
        dist = (ta * (abs(target_deg) + abs(step_deg)) * axle_track_mm * pi) / (tan(radians(abs(step_deg))) * 180)
        print(dist, target_deg, step_deg, self.steer_motor.angle())

        self.steer_motor.run_target(speed_steer, step_deg)
        self._straight(dist, speed_drive, _gyro=False)
        if self.gyro:
            started = False
            print(target_deg, tolerance)
            i = 0
            while abs(self.hub.imu.heading()) - abs(target_deg) > tolerance:
                if i == 0:
                    print(self.hub.imu.heading())
                if not started:
                    self.drive(ta * speed_drive)
                started = True
                self.correct(sa * target_deg, step_deg, pr=i == 0)
                wait(5)
                i += 1
                i %= 50
            # wait(50)
            self.brake()
            print("-------------------------------------------")
            print(self.hub.imu.heading())
            self.hub.imu.reset_heading(self.hub.imu.heading() - sa * target_deg)
            print(self.hub.imu.heading())
            print("-------------------------------------------")
        # self.steer_motor.run_target(speed_steer, saved_angle)
    # def turn(self, target_deg, step_deg, speed_steer=None, speed_drive=None):
    #     if speed_steer is None:
    #         speed_steer = self.default_speed_steer
    #     if speed_drive is None:
    #         speed_drive = self.default_speed
    #     dist = (target_deg * self.axle_track * pi) / (tan(step_deg) * 180)
    #     angle = self.steer_motor.angle()
    #     self.steer_motor.run_target(speed_steer, step_deg)
    #     self.straight(dist * 1000, speed_drive)
    #     self.steer_motor.run_target(speed_steer, angle)
