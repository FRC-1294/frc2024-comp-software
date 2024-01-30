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
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.constants.LauncherConstants.LauncherState;

public class Launcher extends SubsystemBase {
  private final CANSparkMax mIndexer = new CANSparkMax(LauncherConstants.INDEXER_ID, MotorType.kBrushless);

  private final TalonFX mMainFlywheel = new TalonFX(LauncherConstants.MAIN_FLYWHEEL_ID);
  private final TalonFX mRollerFlywheel = new TalonFX(LauncherConstants.ROLLER_FLYWHEEL_ID);

  private double mDesiredVelocityIndexer = 0;

  private double mDesiredVelocityMain = 0;
  private double mDesiredVelocityRoller = 0;

  private LauncherMode mLauncherMode = LauncherMode.OFF;

  private boolean mNoteIndexed = false;

  //toggle to transmit note
  private boolean mIndexToLauncher = false;

  private boolean mLauncherReady = false;

  public Launcher() {
    resetEncoders();
    configureDevices();
  }
  
  public void configureDevices() {
    Slot0Configs slotConfigs = new Slot0Configs();

    slotConfigs.kP = LauncherConstants.LAUNCHER_PID_CONTROLLER.getP();
    slotConfigs.kI = LauncherConstants.LAUNCHER_PID_CONTROLLER.getI();
    slotConfigs.kD = LauncherConstants.LAUNCHER_PID_CONTROLLER.getD();
    slotConfigs.kS = LauncherConstants.LAUNCHER_FF_CONTROLLER.ks;
    slotConfigs.kV = LauncherConstants.LAUNCHER_FF_CONTROLLER.kv;

    mMainFlywheel.getConfigurator().apply(slotConfigs);
    mRollerFlywheel.getConfigurator().apply(slotConfigs);
  }

  @Override
  public void periodic() {
    double actualMainVelocity = mMainFlywheel.getVelocity().getValueAsDouble();
    double actualRollerVelocity = mRollerFlywheel.getVelocity().getValueAsDouble();

    mLauncherReady = Math.abs(Math.abs(mDesiredVelocityMain) - Math.abs(actualMainVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE &&
                     Math.abs(Math.abs(mDesiredVelocityRoller) - Math.abs(actualRollerVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE && 
                     mDesiredVelocityMain != 0 && 
                     mDesiredVelocityRoller != 0;
                      
    runIndexer();
    runLauncher();
  }

  public void runIndexer() {
    if (!mNoteIndexed || !mLauncherReady || !mIndexToLauncher) {
      mDesiredVelocityIndexer = 0;
    }
    else {
      mDesiredVelocityIndexer = LauncherConstants.INDEXER_VELOCITY_DEFAULT;
    }

    mIndexer.set(mDesiredVelocityIndexer);
  }
  
  public void runLauncher() {
    //predicted velocity values
    if (mLauncherMode == LauncherMode.SPEAKER) {
      mDesiredVelocityMain = LauncherState.SPEAKER_DEFAULT.mainVelocity;
      mDesiredVelocityRoller = LauncherState.SPEAKER_DEFAULT.rollerVelocity;
    }
    else if (mLauncherMode == LauncherMode.AMP) {
      mDesiredVelocityMain = LauncherState.AMP_DEFAULT.mainVelocity;
      mDesiredVelocityRoller = LauncherState.AMP_DEFAULT.rollerVelocity;
    }
    else if (mLauncherMode == LauncherMode.TRAP) {
      mDesiredVelocityMain = LauncherState.TRAP_DEFAULT.mainVelocity;
      mDesiredVelocityRoller = LauncherState.TRAP_DEFAULT.rollerVelocity;
    }
    else if (mLauncherMode == LauncherMode.OFF) {
      mDesiredVelocityMain = 0;
      mDesiredVelocityRoller = 0;
    }

    mMainFlywheel.setControl(new VelocityVoltage(mDesiredVelocityMain).withSlot(0));
    mRollerFlywheel.setControl(new VelocityVoltage(mDesiredVelocityMain).withSlot(0));
  }

  public void turnIndexerOn(boolean indexerOn){
    mIndexToLauncher = indexerOn;
  }

  public boolean isIndexerOn() {
    return mIndexToLauncher;
  }

  public boolean isLauncherReady() {
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
