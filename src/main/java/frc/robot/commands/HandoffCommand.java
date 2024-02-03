package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.subsystems.AimingSubsystem;

public class HandoffCommand extends Command{
    private final IntakeSubsystem mIntakeSubsystem;
    private final AimingSubsystem mAimingSubsystem;

    private SequentialCommandGroup command;

    public HandoffCommand(IntakeSubsystem intakeSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mAimingSubsystem);
    }

    @Override
    public void initialize() {
        command = new SequentialCommandGroup(
                mAimingSubsystem.waitUntilSetpoint(AimState.STOW),
                mIntakeSubsystem.getAutomousIntakeCommand());
        command.schedule();
    }

    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Returns true when the command should end.
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
