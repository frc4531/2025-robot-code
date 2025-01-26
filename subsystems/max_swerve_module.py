from rev import SparkMax, SparkMaxConfig, SparkFlex, SparkFlexConfig, SparkBase, ClosedLoopConfig
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants.swerve_constants import ModuleConstants


class MAXSwerveModule:
    def __init__(
        self, driving_can_id: int, turning_can_id: int, chassis_angular_offset: float
    ) -> None:
        """Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
        Encoder.
        """

        self.chassis_angular_offset = 0
        self.desired_state = SwerveModuleState(0.0, Rotation2d())

        self.driving_spark_flex = SparkFlex(
            driving_can_id, SparkFlex.MotorType.kBrushless
        )
        self.turning_spark_max = SparkMax(
            turning_can_id, SparkMax.MotorType.kBrushless
        )

        self.driving_config = SparkFlexConfig()
        self.turning_config = SparkMaxConfig()

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        # self.driving_spark_flex.restoreFactoryDefaults()
        # self.turning_spark_max.restoreFactoryDefaults()

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        self.driving_encoder = self.driving_spark_flex.getEncoder()
        self.turning_encoder = self.turning_spark_max.getAbsoluteEncoder(
            # SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.driving_pid_controller = self.driving_spark_flex.getClosedLoopController()
        self.turning_pid_controller = self.turning_spark_max.getClosedLoopController()
        self.driving_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.turning_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        # Apply position and velocity conversion factors for the driving encoder. The
        # native units for position and velocity are rotations and RPM, respectively,
        # but we want meters and meters per second to use with WPILib's swerve APIs.
        self.driving_config.encoder.positionConversionFactor(
            ModuleConstants.kDrivingEncoderPositionFactor
        )
        self.driving_config.encoder.velocityConversionFactor(
            ModuleConstants.kDrivingEncoderVelocityFactor
        )

        # Apply position and velocity conversion factors for the turning encoder. We
        # want these in radians and radians per second to use with WPILib's swerve
        # APIs.
        self.turning_config.absoluteEncoder.positionConversionFactor(
            ModuleConstants.kTurningEncoderPositionFactor
        )
        self.turning_config.absoluteEncoder.velocityConversionFactor(
            ModuleConstants.kTurningEncoderVelocityFactor
        )

        # Invert the turning encoder, since the output shaft rotates in the opposite direction of
        # the steering motor in the MAXSwerve Module.
        self.turning_config.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)

        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        # to 10 degrees will go through 0 rather than the other direction which is a
        # longer route.
        self.turning_config.closedLoop.positionWrappingEnabled(True)
        self.turning_config.closedLoop.positionWrappingMinInput(
            ModuleConstants.kTurningEncoderPositionPIDMinInput
        )
        self.turning_config.closedLoop.positionWrappingMaxInput(
            ModuleConstants.kTurningEncoderPositionPIDMaxInput
        )

        # Set the PID gains for the driving motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.driving_config.closedLoop.P(ModuleConstants.kDrivingP)
        self.driving_config.closedLoop.I(ModuleConstants.kDrivingI)
        self.driving_config.closedLoop.D(ModuleConstants.kDrivingD)
        self.driving_config.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
        self.driving_config.closedLoop.outputRange(
            ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput
        )

        # Set the PID gains for the turning motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.turning_config.closedLoop.P(ModuleConstants.kTurningP)
        self.turning_config.closedLoop.I(ModuleConstants.kTurningI)
        self.turning_config.closedLoop.D(ModuleConstants.kTurningD)
        self.turning_config.closedLoop.velocityFF(ModuleConstants.kTurningFF)
        self.turning_config.closedLoop.outputRange(
            ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput
        )

        self.driving_config.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.turning_config.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.driving_config.smartCurrentLimit(
            ModuleConstants.kDrivingMotorCurrentLimit
        )
        self.turning_config.smartCurrentLimit(
            ModuleConstants.kTurningMotorCurrentLimit
        )

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations.
        self.driving_spark_flex.configure(self.driving_config,
                                          SparkBase.ResetMode.kResetSafeParameters,
                                          SparkBase.PersistMode.kPersistParameters)
        self.turning_spark_max.configure(self.turning_config,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        self.chassis_angular_offset = chassis_angular_offset
        self.desired_state.angle = Rotation2d(self.turning_encoder.getPosition())
        self.driving_encoder.setPosition(0)

    def get_state(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(
            self.driving_encoder.getVelocity(),
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )

    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(
            self.driving_encoder.getPosition(),
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desired_state: Desired state with speed and angle.

        """
        # Apply chassis angular offset to the desired state.
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle + Rotation2d(
            self.chassis_angular_offset
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        corrected_desired_state.optimize(
            Rotation2d(self.turning_encoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.driving_pid_controller.setReference(
            corrected_desired_state.speed, SparkBase.ControlType.kVelocity
        )
        self.turning_pid_controller.setReference(
            corrected_desired_state.angle.radians(), SparkBase.ControlType.kPosition
        )

        self.desired_state = desired_state

    def reset_encoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.driving_encoder.setPosition(0)
