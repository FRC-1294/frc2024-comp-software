package frc.robot.states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public abstract class MechState {
    protected final LauncherSubsystem mLauncherSubsystem;
    protected final IntakeSubsystem mIntakeSubsystem;
    protected final AimingSubsystem mAimingSubsystem;

    protected MechState (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        mLauncherSubsystem = launcherSubsystem;
        mIntakeSubsystem = intakeSubsystem;
        mAimingSubsystem = aimingSubsystem;
    }

    public void setLauncherSpeed(LauncherMode mode) {}

    public void brakeLauncher() {}

    public void brakeIntake() {
       new InstantCommand(()->mIntakeSubsystem.stopMotors(),mIntakeSubsystem);
    }
    public void runIntakeMotors(){}

    public void preformHandoff(){}

    public void index(){}

    public void launch() {}

    public void controlWrist(double increment) {}

    public void controlElevator(double increment) {}

    public void ClimbExtendedState(AimState state) {}

    public void ClimbExtendedState() {}
    
    public void ClimbRetractedState() {}

    public void setWristSP(AimState state) {}
    
    public void handoffPosition(){}

    public void speakerPosition(){}

    public void ampPosition(){}
    
    public void trapPosition(){}

    public void resetEncoders() {}
}
