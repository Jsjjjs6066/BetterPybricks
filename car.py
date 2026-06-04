from classes import CarDriveBase
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import Motor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

hub = PrimeHub()
print(hub.battery.voltage())

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 13, 200, 200)
car.turn(90, 30)

# steer.run_target(200, 30)
# while True:
#     wait(1000)
