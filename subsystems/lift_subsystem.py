import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from libgrapplefrc import LaserCAN
from rev import SparkFlexConfig, ClosedLoopConfig


class LiftSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_lift_motor = rev.SparkFlex(1, rev.SparkFlex.MotorType.kBrushless)
        self.right_lift_motor = rev.SparkFlex(2, rev.SparkFlex.MotorType.kBrushless)

        self.left_lift_motor.setInverted(True)
        self.right_lift_motor.setInverted(False)

        self.lift_sensor = LaserCAN(1)

        self.lift_limit_switch = wpilib.DigitalInput(1)
        self.lower_limit_switch = wpilib.DigitalInput(2)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        lift_table = nt_instance.getTable("lift_table")

        self.lift_position_entry = lift_table.getDoubleTopic("lift_position").publish()
        self.lift_pid_output_entry = lift_table.getDoubleTopic("lift_pid_output").publish()

        self.lift_limit_switch_entry = lift_table.getBooleanTopic("lift_limit_switch").publish()
        self.lower_limit_switch_entry = lift_table.getBooleanTopic("lower_limit_switch").publish()

        self.left_config = SparkFlexConfig()
        self.right_config = SparkFlexConfig()

        self.left_pid_controller = self.left_lift_motor.getClosedLoopController()
        self.right_pid_controller = self.right_lift_motor.getClosedLoopController()

        self.left_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.right_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)

        self.kP = 0.1
        self.kI = 0.0001
        self.kD = 0
        self.min_speed = -0.9
        self.max_speed = 0.9

        self.left_config.closedLoop.P(self.kP)
        self.left_config.closedLoop.I(self.kI)
        self.left_config.closedLoop.D(self.kD)
        self.left_config.closedLoop.outputRange(self.min_speed, self.max_speed)

        self.right_config.closedLoop.P(self.kP)
        self.right_config.closedLoop.I(self.kI)
        self.right_config.closedLoop.D(self.kD)
        self.right_config.closedLoop.outputRange(self.min_speed, self.max_speed)

    def periodic(self):
        self.lift_position_entry.set(self.get_lift_position())
        self.lift_limit_switch_entry.set(self.get_limit_switch())
        self.lower_limit_switch_entry.set(self.get_lower_limit_switch())

    def set_lift_speed(self, speed):
        # if speed > 0 and self.get_limit_switch():
        #     self.left_lift_motor.set(speed)
        #     self.right_lift_motor.set(speed)
        # elif speed < 0 and self.get_lower_limit_switch():
        #     self.left_lift_motor.set(speed)
        #     self.right_lift_motor.set(speed)
        # else:
        #     self.left_lift_motor.set(0)
        #     self.right_lift_motor.set(0)

        self.left_lift_motor.set(speed)
        self.right_lift_motor.set(speed)

    def get_lift_position(self):
        return self.left_lift_motor.getEncoder().getPosition()

    def get_limit_switch(self):
        return self.lift_limit_switch.get()

    def get_lower_limit_switch(self):
        return self.lower_limit_switch.get()
