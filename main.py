from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

from classes import BetterMotor, BetterDriveBase

hub: PrimeHub = PrimeHub()

left: Motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right: Motor = Motor(Port.A)
hand: BetterMotor = BetterMotor(Port.C, 400)

db: BetterDriveBase = BetterDriveBase(left, right, 62.4, 129)

def test() -> None:
    db.drive(100, 0) # S turn_rate
    wait(1000)
    db.brake()

    db.drive(-100) # Bez turn_rate
    wait(1000)
    db.brake()

    hand.run_target(hand.angle() - 10) # Bez speed (isto kao BetterMotor.run_angle(-10))
    hand.run_target(hand.angle() - 10, 400) # Sa speed (isto kao BetterMotor.run_angle(-10, 400))

    hand.run_angle(10) # Bez speed
    hand.run_angle(10, 400) # Sa speed

if __name__ == "__main__":
    test()