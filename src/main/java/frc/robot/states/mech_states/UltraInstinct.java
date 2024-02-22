package frc.robot.states.mech_states;

import com.fasterxml.jackson.annotation.JsonPropertyOrder;
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

    @Override
    public void setLauncherSpeed(LauncherMode mode) {
        mLauncherSubsystem.setLauncherMode(mode);
    }
    @Override
    public void brakeLauncher() {
        mLauncherSubsystem.stopLauncher();
    }

    @Override
    public void intakeOuterMotor() {
        mIntakeSubsystem.outerMotorAtSpeed(IntakeConstants.OUTER_INTAKE_SPEED_ACTIVE);
    }

    @Override
    public void intakeInnerMotor() {
        mIntakeSubsystem.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_ACTIVE);
    }

    @Override
    public void index(){
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_HANDOFF);
    }

    @Override
    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCH);
    }

    @Override
    public void controlWrist(double increment) {
        mAimingSubsystem.changeDesiredWristRotation(increment);
    }

    @Override
    public void controlElevator(double increment) {
        mAimingSubsystem.changeDesiredElevatorPosition(increment);
    }

    @Override
    public void setElevatorSP(AimState state) {
        mAimingSubsystem.setDesiredElevatorDistance(state.elevatorDistIn);
    }

    @Override
    public void setWristSP(AimState state) {
        mAimingSubsystem.setDesiredWristRotation(state.wristAngleDeg);
    }

    @Override
    public void handoffPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.HANDOFF);
        mLauncherSubsystem.setLauncherMode(LauncherMode.OFF);
    }

    @Override
    public void speakerPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.SPEAKER);
        mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);
    }

    @Override
    public void ampPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.AMP);
        mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);
    }
    
    @Override
    public void trapPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.TRAP);
        mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);
    }
}
