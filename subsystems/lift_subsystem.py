import rev
import wpilib
from commands2 import SubsystemBase
from libgrapplefrc import LaserCAN

class LiftSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_lift_motor = rev.SparkFlex(1, rev.SparkFlex.MotorType.kBrushless)
        self.right_lift_motor = rev.SparkFlex(2, rev.SparkFlex.MotorType.kBrushless)

        self.right_lift_motor.setInverted(True)

        self.lift_sensor = LaserCAN(1)

    # def periodic(self):
        # wpilib.SmartDashboard.putNumber("Lift Sensor", self.get_distance())

    def set_lift_speed(self, speed):
        self.left_lift_motor.set(speed)
        self.right_lift_motor.set(speed)

    # def get_distance(self):
        # measurement = self.lift_sensor.get_measurement()
        # return measurement.distance_mm
