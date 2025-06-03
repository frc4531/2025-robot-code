import wpilib.sysid
import rev
import subsystems.lift_subsystem


dynamic_forward_state = wpilib.sysid.State.kDynamicForward
test = wpilib.sysid.SysIdRoutineLog.MotorLog.position(self, subsystems.lift_subsystem.LiftSubsystem.get_lift_position())