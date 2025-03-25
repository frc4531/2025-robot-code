import math

import commands2
import ntcore
import wpilib
import libgrapplefrc

from commands2 import cmd, WaitCommand
from commands2.cmd import waitSeconds
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.climber_down import ClimberDown
from commands.climber_up import ClimberUp
from commands.drive_to_center_reef import DriveToCenterReef
from commands.drive_to_coral_station import DriveToCoralStation
from commands.drive_to_left_reef import DriveToLeftReef
from commands.drive_to_right_reef import DriveToRightReef
from commands.input_drive import InputDrive
from commands.intake_algae import IntakeAlgae
from commands.lift_stop import LiftStop
from constants.position_constants import PositionConstants
from commands.drive_command import DriveCommand
from commands.intake_out import IntakeOut
from commands.intake_until_loaded import IntakeUntilLoaded
from commands.lift_down import LiftDown
from commands.lift_to_position import LiftToPosition
from commands.lift_up import LiftUp
from commands.swing_arm_down import SwingArmDown
from commands.swing_arm_to_position import SwingArmToPosition
from commands.swing_arm_up import SwingArmUp
from commands.drive_flip import DriveFlip
from commands.wrist_down import WristDown
from commands.wrist_to_position import WristToPosition
from commands.wrist_up import WristUp
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.lift_subsystem import LiftSubsystem
from subsystems.wrist_subsystem import WristSubsystem
from subsystems.swing_arm_subsystem import SwingArmSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.climber_subsystem import ClimberSubsystem


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
        self.climber_subsystem = ClimberSubsystem()

        libgrapplefrc.can_bridge_tcp()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )
        # Configure Auto Chooser
        self.chooser = wpilib.SendableChooser()
        self.do_nothing = "Do Nothing"
        self.drive_forward = "Drive Forward"
        self.mid_one_coral = "Middle One Coral"

        self.chooser.setDefaultOption("Shoot 1 Only", self.do_nothing)
        self.chooser.addOption("Drive Forward", self.drive_forward)
        self.chooser.addOption("Middle One Coral", self.mid_one_coral)


    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        #commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
        #    SwingArmToPosition(self.swing_arm_subsystem,0.4)
        #)
        #commands2.button.JoystickButton(self.operator_controller, 2).whileTrue(
        #    SwingArmToPosition(self.swing_arm_subsystem, 0.6)
        #)
        #commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
        #    SwingArmToPosition(self.swing_arm_subsystem, 0.8)
        #)
        commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
            LiftUp(self.lift_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            LiftDown(self.lift_subsystem)
        )

        #commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
        #    SwingArmUp(self.swing_arm_subsystem)
        #)
        #commands2.button.JoystickButton(self.operator_controller, 4).whileTrue(
        #   SwingArmDown(self.swing_arm_subsystem)
        #)

        #commands2.button.JoystickButton(self.operator_controller, 5).whileTrue(
        #    WristUp(self.wrist_subsystem)
        #)
        #commands2.button.JoystickButton(self.operator_controller, 6).whileTrue(
        #    WristDown(self.wrist_subsystem)
        #)

        #commands2.button.JoystickButton(self.operator_controller, 4).whileTrue(
        #    WristToPosition(self.wrist_subsystem, 0.4)
        #)
        # Start Operator Control Block
        # Intake In
        commands2.button.JoystickButton(self.operator_controller, 6).onTrue(
            IntakeUntilLoaded(self.intake_subsystem)
        )
        # Intake Out
        commands2.button.JoystickButton(self.operator_controller, 5).whileTrue(
            IntakeOut(self.intake_subsystem)
        )
        # Intake From Coral Station
        commands2.button.JoystickButton(self.operator_controller, 2).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralIntakeLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 2).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralIntakeWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 2).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralIntakeSwingArm)

        )
        # Level 1 Coral Deposit
        commands2.button.JoystickButton(self.operator_controller, 13).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralOneLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 13).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralOneWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 13).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralOneSwingArm)
        )
        # Level 2 Coral Deposit
        commands2.button.JoystickButton(self.operator_controller, 12).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralTwoLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 12).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralTwoWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 12).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralTwoSwingArm)
        )
        # Level 3 Coral Deposit
        commands2.button.JoystickButton(self.operator_controller, 8).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralThreeLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 8).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralThreeWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 8).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralThreeSwingArm)
        )
        # Level 4 Deposit
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralFourLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralFourWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralFourSwingArm)
        )
        # Ground Algae
        commands2.button.JoystickButton(self.operator_controller, 9).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kAlgaeGroundLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 9).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kAlgaeGroundWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 9).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kAlgaeGroundSwingArm)
        )
        # Level 2 Algae
        commands2.button.JoystickButton(self.operator_controller, 10).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kAlgaeTwoLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 10).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kAlgaeTwoWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 10).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kAlgaeTwoSwingArm)
        )
        # Level 3 Algae
        commands2.button.JoystickButton(self.operator_controller, 11).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kAlgaeThreeLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 11).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kAlgaeThreeWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 11).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kAlgaeThreeSwingArm)
        )
        # Algae Processor
        commands2.button.JoystickButton(self.operator_controller, 4).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kAlgaeProcessorLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 4).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kAlgaeProcessorWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 4).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kAlgaeProcessorSwingArm)
        )
        # START DRIVER BLOCK
        # Stowed Position
        commands2.button.JoystickButton(self.driver_controller, 1).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kStoragePosLift)
        )
        commands2.button.JoystickButton(self.driver_controller, 1).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kStoragePosWrist)
        )
        commands2.button.JoystickButton(self.driver_controller, 1).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kStoragePosSwingArm)
        )
        # Left Reef Align
        commands2.button.JoystickButton(self.driver_controller, 3).whileTrue(
            DriveToLeftReef(self.drive_subsystem, self.vision_subsystem, self.driver_controller)
        )
        # Center Reef Align
        commands2.button.JoystickButton(self.driver_controller, 2).whileTrue(
            DriveToCenterReef(self.drive_subsystem, self.vision_subsystem, self.driver_controller)
        )
        # Right Reef Align
        commands2.button.JoystickButton(self.driver_controller, 4).whileTrue(
            DriveToRightReef(self.drive_subsystem, self.vision_subsystem, self.driver_controller)
        )
        # Coral Station Align
        commands2.button.JoystickButton(self.driver_controller, 6).whileTrue(
            DriveToCoralStation(self.drive_subsystem, self.vision_subsystem, self.driver_controller)
        )
        # LIFT EMERGENCY STOP
        commands2.button.JoystickButton(self.driver_controller, 9).whileTrue(
            LiftStop(self.lift_subsystem)
        )
        # Climber Up
        commands2.button.JoystickButton(self.driver_controller, 5).whileTrue(
            ClimberUp(self.climber_subsystem)
        )
        # Climber Down
        commands2.button.JoystickButton(self.driver_controller, 10).whileTrue(
            ClimberDown(self.climber_subsystem)
        )

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        )
        commands2.button.JoystickButton(self.operator_controller, 8).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralThreeSwingArm)
        )
        # Level 4 Coral Deposit
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralFourLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralFourWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralFourSwingArm)
        )
        # Ground Algae
        commands2.button.JoystickButton(self.operator_controller, 1).onTrue(
        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # Forward Auto Trajectory to follow. All units in meters.
        forward_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -3, Rotation2d.fromDegrees(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        thetaController)

        # forward_trajectory_command = commands2.SwerveControllerCommand(
        #     forward_trajectory,
        #     self.drive_subsystem.get_pose,  # Functional interface to feed supplier
        #     DriveConstants.kDriveKinematics,
        #     # Position controllers
        #     holonomic_controller,
        #     self.drive_subsystem.set_module_states(),
        #     (self.drive_subsystem,),
        # )

        # Start Auto Logic
        return commands2.ParallelDeadlineGroup(
            WaitCommand(1),
            InputDrive(self.drive_subsystem, 0.4, 0, 0)
        )

        auto_selected = self.drive_forward# self.chooser.getSelected()

        match auto_selected:
            case self.do_nothing:
                return waitSeconds(1)
            case self.drive_forward:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(4),
                        commands2.RunCommand(lambda: self.drive_subsystem.drive(0, 0.4, 0, True, False))
                    )
                )
            case self.mid_one_coral:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(5),
                        LiftToPosition(self.lift_subsystem, PositionConstants.kCoralOneLift),
                        SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralOneSwingArm),
                        WristToPosition(self.wrist_subsystem, PositionConstants.kCoralOneWrist)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1),
                        commands2.RunCommand(lambda: self.drive_subsystem.drive(0, -0.4, 0, True, False))
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1),
                        IntakeOut(self.intake_subsystem)
                    ),
                )

