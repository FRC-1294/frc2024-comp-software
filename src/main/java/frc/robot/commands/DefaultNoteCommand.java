// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultNoteCommand extends Command {
    private final IntakeSubsystem mIntakeSubsystem;    
    private final LauncherSubsystem mLauncherSubsystem;
    private final AimingSubsystem mAimingSubsystem;

    private LauncherMode mLauncherMode;
    private AimState mAimState;

    public DefaultNoteCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem, LauncherMode launcherMode, AimState aimState) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        mLauncherMode = launcherMode;
        mAimState = aimState;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (Input.getY()) { 
            mLauncherMode = LauncherMode.OFF;
        }
        //speaker
        else if (Input.getA()) { 
            mLauncherMode = LauncherMode.SPEAKER;
            mAimState = AimState.SPEAKER;
        } 
        //amp
        else if (Input.getB()) { 
            mLauncherMode = LauncherMode.AMP;
            mAimState = AimState.AMP;
        }
        //trap
        else if (Input.getX()) { 
            mLauncherMode = LauncherMode.TRAP;
            //mAimState = AimState.TRAP;
        }
        if (Input.getRightBumper()) { 
            mLauncherSubsystem.turnIndexerOn(!mLauncherSubsystem.isIndexerOn());
        }
        if (Input.getLeftBumper()) {
            HandoffCommand command = new HandoffCommand(mIntakeSubsystem, mAimingSubsystem);
            command.schedule(); 
        }

        mLauncherSubsystem.setLauncherMode(mLauncherMode);
        mAimingSubsystem.setDesiredSetpoint(mAimState);

        if (Math.abs(Input.getLeftStickY()) > 0) {
            //convert between input to increment
            double increment = Input.getLeftStickY() * AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT;
            mAimingSubsystem.changeDesiredWristRotation(increment);
        }
        
        if (Math.abs(Input.getRightStickY()) > 0) {
            //convert between input to increment
            double increment = Input.getRightStickY() * AimingConstants.MAX_WRIST_TELEOP_INCREMENT;
            mAimingSubsystem.changeDesiredElevatorPosition(increment);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Returns true when the command should end.
    }

    @Override
    public boolean isFinished() {
        return mAimingSubsystem.atSetpoints();
    }
}
