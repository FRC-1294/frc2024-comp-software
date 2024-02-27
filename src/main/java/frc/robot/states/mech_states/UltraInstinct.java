package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class UltraInstinct extends MechState {
    public UltraInstinct (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public void setLauncherSpeed(LauncherMode mode) {
        new InstantCommand(()->mLauncherSubsystem.setLauncherMode(mode),mLauncherSubsystem).schedule();
    }
    @Override
    public void brakeLauncher() {
        new InstantCommand(()->mLauncherSubsystem.stopLauncher(),mLauncherSubsystem).schedule();
    }

    @Override
    public void runIntakeMotors() {
        new InstantCommand(() -> {
        mIntakeSubsystem.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_ACTIVE);
        mIntakeSubsystem.outerMotorAtSpeed(IntakeConstants.OUTER_INTAKE_SPEED_ACTIVE);},mIntakeSubsystem).schedule();
    }

    @Override
    public void index(){
        new InstantCommand(() -> mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_INDEXING),mLauncherSubsystem).schedule();;
    }

    @Override
    public void launch() {
        new InstantCommand(() -> mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCHING),mLauncherSubsystem).schedule();;
    }

    @Override
    public void controlWrist(double increment) {
        new InstantCommand(()->mAimingSubsystem.changeDesiredWristRotation(increment),mAimingSubsystem).schedule();;
    }

    @Override
    public void controlElevator(double increment) {
        new InstantCommand(()->mAimingSubsystem.changeDesiredElevatorPosition(increment),mAimingSubsystem).schedule();;
    }

    @Override
    public void ClimbExtendedState() {
        new InstantCommand(()->mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB_UP)).schedule();
    }

    @Override
    public void ClimbRetractedState() {
        new InstantCommand(()->mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB_DOWN)).schedule();
    }

    @Override
    public void setWristSP(AimState state) {
        new InstantCommand(()->mAimingSubsystem.setDesiredWristRotation(state.mWristAngleDegrees, state.mWristToleranceDegrees),mAimingSubsystem).schedule();;
    }

    @Override
    public void preformHandoff(){
        mHandoffPositionCommand.schedule();
    }

    @Override
    public void speakerPosition(){
        mSpeakerPositionCommand.schedule();
    }

    @Override
    public void ampPosition(){
        mAmpPositionCommand.schedule();
    }
    
    @Override
    public void trapPosition(){
        mTrapPositionCommand.schedule();
    }

    @Override
    public void handoffPosition(){
        mHandoffPositionCommand.schedule();
    }
}
