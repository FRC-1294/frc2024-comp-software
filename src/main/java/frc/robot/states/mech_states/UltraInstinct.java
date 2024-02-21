package frc.robot.states.mech_states;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class UltraInstinct extends MechState {
    public UltraInstinct (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    public void setLauncherSpeed(LauncherMode mode) {
        mLauncherSubsystem.setLauncherMode(mode);
    }

    public void brakeLauncher() {
        mLauncherSubsystem.stopLauncher();
    }

    public void intake() {
        mIntakeSubsystem.intakeAtSpeed(IntakeConstants.INTAKE_SPEED);
    }

    public void index(){
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_HANDOFF);
    }

    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCH);
    }

    public void controlWrist(double increment) {
        mAimingSubsystem.changeDesiredWristRotation(increment);
    }

    public void controlElevator(double increment) {
        mAimingSubsystem.changeDesiredElevatorPosition(increment);
    }

    public void setElevatorSP(AimState state) {
        mAimingSubsystem.setDesiredElevatorDistance(state.elevatorDistIn);
    }

    public void setWristSP(AimState state) {
        mAimingSubsystem.setDesiredWristRotation(state.wristAngleDeg);
    }

    public void handoffPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.HANDOFF);
        mLauncherSubsystem.setLauncherMode(LauncherMode.OFF);
    }

    public void speakerPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.SPEAKER);
        mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);
    }

    public void ampPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.AMP);
        mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);
    }
    
    public void trapPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.TRAP);
        mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);
    }
}
