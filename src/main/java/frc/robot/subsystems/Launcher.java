// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;



public class Launcher extends SubsystemBase {

  private final CANSparkMax mIndexer = new CANSparkMax(LauncherConstants.INDEXER_ID, MotorType.kBrushless);

  private final TalonFX mMainFlywheel = new TalonFX(LauncherConstants.MAIN_FLYWHEEL_ID);
  private final TalonFX mRollerFlywheel = new TalonFX(LauncherConstants.ROLLER_FLYWHEEL_ID);

  public enum LauncherMode {
    SPEAKER, AMP, OFF;
  }

  LauncherMode mLauncherMode = LauncherMode.OFF;

  double mSetVelocityIndexer = 0;

  double mSetVelocityMain = 0;
  double mSetVelocityRoller = 0;

  boolean mNoteIndexed = false;

  //toggle to transmit note
  boolean mIndexToShooter = false;

  boolean mLauncherReady = false;


  public Launcher() {
    resetEncoders();

    Slot0Configs slotConfigs = new Slot0Configs(); //very confused

    slotConfigs.kV = LauncherConstants.LAUNCHER_MAIN_PID[0];
    slotConfigs.kP = LauncherConstants.LAUNCHER_MAIN_PID[1];
    slotConfigs.kI = LauncherConstants.LAUNCHER_MAIN_PID[2];
    slotConfigs.kD = LauncherConstants.LAUNCHER_MAIN_PID[3];


    mMainFlywheel.getConfigurator().apply(slotConfigs);
    mRollerFlywheel.getConfigurator().apply(slotConfigs);

    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    mMainFlywheel.setControl(request.withVelocity(0).withFeedForward(0));
    mRollerFlywheel.setControl(request.withVelocity(0).withFeedForward(0));
  }

  @Override
  public void periodic() {
      double actualMainVelocity = mMainFlywheel.getVelocity().getValueAsDouble();
      double actualRollerVelocity = mRollerFlywheel.getVelocity().getValueAsDouble();

      double expectedMainVelocity = mSetVelocityMain * LauncherConstants.FLYWHEEL_MAX_VELOCITY;
      double expectedRollerVelocity = mSetVelocityMain * LauncherConstants.FLYWHEEL_MAX_VELOCITY;

      mLauncherReady = Math.abs(Math.abs(expectedMainVelocity) - Math.abs(actualMainVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE &&
                       Math.abs(Math.abs(expectedRollerVelocity) - Math.abs(actualRollerVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE && 
                       mSetVelocityMain != 0 && 
                       mSetVelocityRoller != 0;
      runIndexer();
      runLauncher();
  }

  public void runIndexer() {
    if (!mNoteIndexed || !mLauncherReady || !mIndexToShooter) {
      mSetVelocityIndexer = 0;
    }
    else {
      mSetVelocityIndexer = 1; //TBD
    }

    mIndexer.set(mSetVelocityIndexer);
  }


  //runLauncher
  public void runLauncher() {

    //predicted velocity values
    if (mLauncherMode == LauncherMode.SPEAKER) {
      mSetVelocityMain = 1;
      mSetVelocityRoller = -1;
    }
    else if (mLauncherMode == LauncherMode.AMP) {
      mSetVelocityMain = 0.1; //TBD
      mSetVelocityRoller = -0.1;
    }
    else if (mLauncherMode == LauncherMode.OFF) {
      mSetVelocityMain = 0;
      mSetVelocityRoller = 0;
    }

    mMainFlywheel.set(mSetVelocityMain);
    mRollerFlywheel.set(mSetVelocityRoller);
  }

  public void turnIndexerOn(boolean indexerOn){
    mIndexToShooter = indexerOn;
  }

  public boolean getLauncherReady() {
    return mLauncherReady;
  }

  public void setIsNoteIndexed(boolean containsNote) {
    mNoteIndexed = containsNote;
  }

  public void setLauncherMode(LauncherMode mode) {
    mLauncherMode = mode;
    if (mode == LauncherMode.OFF) {
      runLauncher();
    }
  }
  
  public void stopLauncher() {
    mLauncherMode = LauncherMode.OFF;
    runLauncher();
  }

  private void resetEncoders() {
    mMainFlywheel.setPosition(0);
    mRollerFlywheel.setPosition(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
