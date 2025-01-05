from rev import CANSparkMax, SparkMaxAbsoluteEncoder, CANSparkFlex
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

        self.driving_spark_flex = CANSparkFlex(
            driving_can_id, CANSparkFlex.MotorType.kBrushless
        )
        self.turning_spark_max = CANSparkMax(
            turning_can_id, CANSparkMax.MotorType.kBrushless
        )

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        self.driving_spark_flex.restoreFactoryDefaults()
        self.turning_spark_max.restoreFactoryDefaults()

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        self.driving_encoder = self.driving_spark_flex.getEncoder()
        self.turning_encoder = self.turning_spark_max.getAbsoluteEncoder(
            SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.driving_pid_controller = self.driving_spark_flex.getPIDController()
        self.turning_pid_controller = self.turning_spark_max.getPIDController()
        self.driving_pid_controller.setFeedbackDevice(self.driving_encoder)
        self.turning_pid_controller.setFeedbackDevice(self.turning_encoder)

        # Apply position and velocity conversion factors for the driving encoder. The
        # native units for position and velocity are rotations and RPM, respectively,
        # but we want meters and meters per second to use with WPILib's swerve APIs.
        self.driving_encoder.setPositionConversionFactor(
            ModuleConstants.kDrivingEncoderPositionFactor
        )
        self.driving_encoder.setVelocityConversionFactor(
            ModuleConstants.kDrivingEncoderVelocityFactor
        )

        # Apply position and velocity conversion factors for the turning encoder. We
        # want these in radians and radians per second to use with WPILib's swerve
        # APIs.
        self.turning_encoder.setPositionConversionFactor(
            ModuleConstants.kTurningEncoderPositionFactor
        )
        self.turning_encoder.setVelocityConversionFactor(
            ModuleConstants.kTurningEncoderVelocityFactor
        )

        # Invert the turning encoder, since the output shaft rotates in the opposite direction of
        # the steering motor in the MAXSwerve Module.
        self.turning_encoder.setInverted(ModuleConstants.kTurningEncoderInverted)

        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        # to 10 degrees will go through 0 rather than the other direction which is a
        # longer route.
        self.turning_pid_controller.setPositionPIDWrappingEnabled(True)
        self.turning_pid_controller.setPositionPIDWrappingMinInput(
            ModuleConstants.kTurningEncoderPositionPIDMinInput
        )
        self.turning_pid_controller.setPositionPIDWrappingMaxInput(
            ModuleConstants.kTurningEncoderPositionPIDMaxInput
        )

        # Set the PID gains for the driving motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.driving_pid_controller.setP(ModuleConstants.kDrivingP)
        self.driving_pid_controller.setI(ModuleConstants.kDrivingI)
        self.driving_pid_controller.setD(ModuleConstants.kDrivingD)
        self.driving_pid_controller.setFF(ModuleConstants.kDrivingFF)
        self.driving_pid_controller.setOutputRange(
            ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput
        )

        # Set the PID gains for the turning motor. Note these are example gains, and you
        # may need to tune them for your own robot!
        self.turning_pid_controller.setP(ModuleConstants.kTurningP)
        self.turning_pid_controller.setI(ModuleConstants.kTurningI)
        self.turning_pid_controller.setD(ModuleConstants.kTurningD)
        self.turning_pid_controller.setFF(ModuleConstants.kTurningFF)
        self.turning_pid_controller.setOutputRange(
            ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput
        )

        self.driving_spark_flex.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.turning_spark_max.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.driving_spark_flex.setSmartCurrentLimit(
            ModuleConstants.kDrivingMotorCurrentLimit
        )
        self.turning_spark_max.setSmartCurrentLimit(
            ModuleConstants.kTurningMotorCurrentLimit
        )

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations.
        self.driving_spark_flex.burnFlash()
        self.turning_spark_max.burnFlash()

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
        optimized_desired_state = SwerveModuleState.optimize(
            corrected_desired_state, Rotation2d(self.turning_encoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.driving_pid_controller.setReference(
            optimized_desired_state.speed, CANSparkFlex.ControlType.kVelocity
        )
        self.turning_pid_controller.setReference(
            optimized_desired_state.angle.radians(), CANSparkMax.ControlType.kPosition
        )

        self.desired_state = desired_state

    def reset_encoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.driving_encoder.setPosition(0)
