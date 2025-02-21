import ntcore
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

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        lift_table = nt_instance.getTable("lift_table")

        self.lift_position_entry = lift_table.getDoubleTopic("lift_position").publish()
        self.lift_pid_output_entry = lift_table.getDoubleTopic("lift_pid_output").publish()

    def periodic(self):
        self.lift_position_entry.set(self.get_lift_position())

    def set_lift_speed(self, speed):
        self.left_lift_motor.set(speed)
        self.right_lift_motor.set(speed)

    def get_lift_position(self):
        millimeter = self.lift_sensor.get_measurement().distance_mm
        inch = (millimeter/25.4)
        return inch
