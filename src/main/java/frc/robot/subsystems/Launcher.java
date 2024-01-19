// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LauncherConstants;



public class Launcher extends SubsystemBase {

  private final CANSparkMax mIndexer = new CANSparkMax(LauncherConstants.INDEXER_ID, MotorType.kBrushless);

  private final TalonFX mMainFlywheel = new TalonFX(LauncherConstants.MAIN_FLYWHEEL_ID);
  private final TalonFX mRollerFlywheel = new TalonFX(LauncherConstants.ROLLER_FLYWHEEL_ID);

  private final DutyCycleEncoder mAbsFlywheelEncoder = new DutyCycleEncoder(LauncherConstants.ABS_LAUNCHER_ENCODER_ID); 
  //are there two encoders for the two flywheels TT
  //private final RelativeEncoder mIndexerEncoder //is there an encoder for indexer

  public enum LauncherMode {
    SPEAKER, AMP, OFF;
  }

  LauncherMode mLauncherMode = LauncherMode.OFF;

  double mSetVelocityIndexer = 0;

  double mSetVelocityMain = 0;
  double mSetVelocityRoller = 0;

  boolean mNoteIndexed = false;

  boolean mLauncherReady = false;


  public Launcher() {
    resetEncoders();

    var slot0Configs = new Slot0Configs(); //very confused

    slot0Configs.kS = LauncherConstants.LAUNCHER_MAIN_PID[0];
    slot0Configs.kV = LauncherConstants.LAUNCHER_MAIN_PID[1];
    slot0Configs.kP = LauncherConstants.LAUNCHER_MAIN_PID[2];
    slot0Configs.kI = LauncherConstants.LAUNCHER_MAIN_PID[3];
    slot0Configs.kD = LauncherConstants.LAUNCHER_MAIN_PID[4];

    mMainFlywheel.getConfigurator().apply(slot0Configs);
    mRollerFlywheel.getConfigurator().apply(slot0Configs);

    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    mMainFlywheel.setControl(request.withVelocity(0).withFeedForward(0));
    mRollerFlywheel.setControl(request.withVelocity(0).withFeedForward(0));
  }

  @Override
  public void periodic() {
      //how do i convert encoder values to a value between -1 and 1
      double actualVelocityFlywheel = mAbsFlywheelEncoder.getFrequency() / LauncherConstants.ENCODER_FLYWHEEL_INCREMENT * 60; //in HZ per second
      mLauncherReady = Math.abs(Math.abs(mSetVelocityMain) - Math.abs(actualVelocityFlywheel)) <= LauncherConstants.FLYWHEEL_TOLERANCE &&
                    Math.abs(Math.abs(mSetVelocityRoller) - Math.abs(mAbsFlywheelEncoder.getFrequency())) <= LauncherConstants.FLYWHEEL_TOLERANCE && 
                    mSetVelocityMain != 0 && 
                    mSetVelocityRoller != 0;
      runIndexer();
      runLauncher();
  }

  public void runIndexer() {
    if (mNoteIndexed || !mLauncherReady) {
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
      mSetVelocityMain = 1; //TBD
      mSetVelocityRoller = -1; //all TBD
    }
    else if (mLauncherMode == LauncherMode.AMP) {
      mSetVelocityMain = 0.1;
      mSetVelocityRoller = -0.1;
    }
    else if (mLauncherMode == LauncherMode.OFF) {
      mSetVelocityMain = 0;
      mSetVelocityRoller = 0;
    }

    mMainFlywheel.set(mSetVelocityMain);
    mRollerFlywheel.set(mSetVelocityRoller);
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
  
  public void stopShooter() {
    mLauncherMode = LauncherMode.OFF;
    runLauncher();
  }

  private void resetEncoders() {
    mMainFlywheel.setPosition(0);
    mRollerFlywheel.setPosition(0);

    mAbsFlywheelEncoder.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
