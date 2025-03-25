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

        self.lift_limit_switch = wpilib.DigitalInput(1)
        self.lower_limit_switch = wpilib.DigitalInput(2)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        lift_table = nt_instance.getTable("lift_table")

        self.lift_position_entry = lift_table.getDoubleTopic("lift_position").publish()
        self.lift_pid_output_entry = lift_table.getDoubleTopic("lift_pid_output").publish()

        self.lift_limit_switch_entry = lift_table.getBooleanTopic("lift_limit_switch").publish()
        self.lower_limit_switch_entry = lift_table.getBooleanTopic("lower_limit_switch").publish()

    def periodic(self):
        self.lift_position_entry.set(self.get_lift_position())
        self.lift_limit_switch_entry.set(self.get_limit_switch())
        self.lower_limit_switch_entry.set(self.get_lower_limit_switch())

    def set_lift_speed(self, speed):
        if speed > 0 and self.get_limit_switch():
            self.left_lift_motor.set(speed)
            self.right_lift_motor.set(speed)
        elif speed < 0 and self.get_lower_limit_switch():
            self.left_lift_motor.set(speed)
            self.right_lift_motor.set(speed)
        else:
            self.left_lift_motor.set(0)
            self.right_lift_motor.set(0)

    def get_lift_position(self):
        millimeter = self.lift_sensor.get_measurement().distance_mm
        inch = (millimeter/25.4)
        return inch

    def get_limit_switch(self):
        return self.lift_limit_switch.get()

    def get_lower_limit_switch(self):
        return self.lower_limit_switch.get()
