import math

import commands2
import wpilib

from commands2 import cmd
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.drive_command import DriveCommand
from commands.lift_down import LiftDown
from commands.lift_to_position import LiftToPosition
from commands.lift_up import LiftUp
from commands.swing_arm_to_position import SwingArmToPosition
from commands.wrist_to_position import WristToPosition
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.lift_subsystem import LiftSubsystem
from subsystems.wrist_subsystem import WristSubsystem
from subsystems.swing_arm_subsystem import SwingArmSubsystem
from subsystems.intake_subsystem import IntakeSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.vision_subsystem = VisionSubsystem()
        self.lift_subsystem = LiftSubsystem()
        self.swing_arm_subsystem = SwingArmSubsystem()
        self.wrist_subsystem = WristSubsystem()
        self.intake_subsystem = IntakeSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )



    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
            LiftDown(self.lift_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 2).whileTrue(
            LiftUp(self.lift_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            LiftToPosition(self.lift_subsystem, 0.4)
        )

        commands2.button.JoystickButton(self.operator_controller, 4).whileTrue(
            WristToPosition(self.wrist_subsystem, 0.4)
        )

        commands2.button.JoystickButton(self.operator_controller, 5).whileTrue(
            SwingArmToPosition(self.swing_arm_subsystem, 0.70)
        )
        commands2.button.JoystickButton(self.operator_controller, 6).onTrue(
            LiftToPosition(self.lift_subsystem, 24)
        )

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        example_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(1, 1), Translation2d(2, -1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(3, 0, Rotation2d(0)),
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        theta_controller)

        swerve_controller_command = commands2.SwerveControllerCommand(
            example_trajectory,
            self.drive_subsystem.get_pose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.drive_subsystem.set_module_states,  # Same error appears in 2024-robot-code so IDK
            (self.drive_subsystem,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.drive_subsystem.reset_odometry(example_trajectory.initialPose())

        # Run path following command, then stop at the end.
        # return self.test_path_auto.andThen(
        #     cmd.run(
        #         lambda: self.drive_subsystem.drive(0, 0, 0, False, False),
        #         self.drive_subsystem
        #     )
        # )

        return swerve_controller_command.andThen(
            cmd.run(
                lambda: self.drive_subsystem.drive(0, 0, 0, False, False),
                self.drive_subsystem,
            )
        )
