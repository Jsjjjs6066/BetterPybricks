from classes import BetterMotor, CarDriveBase
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait, multitask, run_task

hub = PrimeHub()
print(hub.battery.voltage())

drive = Motor(Port.F, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D)

car = CarDriveBase(drive, steer, 62.4, 13, 450, 300, hub)

left_sensor = UltrasonicSensor(Port.E)
right_sensor = UltrasonicSensor(Port.C)
color_sensor = ColorSensor(Port.B)
front_sensor = UltrasonicSensor(Port.A)

def gumb():
    pressed = []
    while not any(pressed):
        pressed = hub.buttons.pressed()
        wait(10)

wall = 1100
turn_r = 330

# st = 0  # 1
# st = 233  # 2
st = 450  # 3

# car.turn(90, 30)
# car.straight(300, use_gyro=True)
steer.run_target(300, 0)
steer.reset_angle(0)
car.use_gyro(True)
car.drive()
while front_sensor.distance() > wall:
    car.correct()
    wait(5)
car.brake()
dist = car.distance()
print(dist)
car.straight(st)
car.turn_radius(90, turn_r)

for i in range(11):
    car.drive()
    while front_sensor.distance() > wall:
        car.correct()
        wait(5)
    car.brake()
    car.straight(st)
    car.turn_radius(90, turn_r)

car.drive()
while front_sensor.distance() > wall + dist:
    car.correct()
    wait(5)
car.brake()

# for i in range(4):
#     car.turn(90, 30)
#     hub.speaker.beep()
#     car.straight(2250)
#     hub.speaker.beep()
# while True:
#     car.correct()
#     wait(5)
