import rev
import wpilib
from commands2 import SubsystemBase


class LiftSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_lift_motor = rev.SparkFlex(1, rev.SparkFlex.MotorType.kBrushless)
        self.right_lift_motor = rev.SparkFlex(2, rev.SparkFlex.MotorType.kBrushless)

        self.right_lift_motor.setInverted(True)

    def set_lift_speed(self, speed):
        self.left_lift_motor.set(speed)
        self.right_lift_motor.set(speed)