from classes import BetterMotor, CarDriveBase
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import Motor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait, multitask, run_task

hub = PrimeHub()
print(hub.battery.voltage())

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 13, 450, 300, hub)
# car.turn(90, 30)
# car.straight(300, use_gyro=True)
steer.run_target(300, 0)
steer.reset_angle(0)
car.use_gyro(True)

for i in range(4):
    car.turn(90, 30)
    hub.speaker.beep()
    car.straight(2250)
    hub.speaker.beep()
# while True:
#     car.correct()
#     wait(5)
