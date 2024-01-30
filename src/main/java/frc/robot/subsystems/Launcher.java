// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
<<<<<<< HEAD
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LauncherConstants;



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

      double expectedVelocity = mSetVelocityMain * LauncherConstants.FLYWHEEL_MAX_VELOCITY;

      mLauncherReady = Math.abs(Math.abs(expectedVelocity) - Math.abs(actualMainVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE &&
                       Math.abs(Math.abs(expectedVelocity) - Math.abs(actualRollerVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE && 
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
  
  public void stopLauncher() {
    mLauncherMode = LauncherMode.OFF;
    runLauncher();
  }

  private void resetEncoders() {
    mMainFlywheel.setPosition(0);
    mRollerFlywheel.setPosition(0);
  }

=======


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Launcher extends SubsystemBase {
  //Declare joysticks
  Joystick joysticks;
  //Declare mainTalon, rollerTalon
  TalonFX mainTalon = TalonFX(IDs.mainFlyWheelID); //need ID file?
  TalonFX rollerTalon = TalonFX(IDs.rollerFlyWheelID);

  //Create launcherModeIndex, set to 0
  int launcherModeIndex = 0;
  //Create new instance of launcherMode and set to off
  LauncherMode launcherMode = LauncherMode.off;
  //Create launcherReady boolean, meaning velocity is within tolerance, and set to false
  boolean launcherReady = false;
  //Create launcherScrollPressed boolean, meaning launcher is enabled, and set to false
  boolean launcherScrollPressed = false;

  double setVelocityMain = 0;
  double setVelocityRoller = 0;

  final LauncherMode[] modes = new LauncherMode[]
  {LauncherMode.speaker, LauncherMode.amp, LauncherMode.off};

  public Launcher(Joystick joysticks) {
    //launcher method (joysticks)
    this.joysticks = joysticks;

    var slot0Configs = new Slot0Configs();

    slot0Configs.kS = Constants.kLauncherMainPID[0];
    slot0Configs.kV = Constants.kLauncherMainPID[1];
    slot0Configs.kP = Constants.kLauncherMainPID[2];
    slot0Configs.kI = Constants.kLauncherMainPID[3];
    slot0Configs.kD = Constants.kLauncherMainPID[4];

    mainTalon.getConfigurator().apply(slot0Configs);
    rollerTalon.getConfigurator().apply(slot0Configs);

    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    mainTalon.setControl(request.withVelocity(8).withFeedForward(0.5));
    rollerTalon.setControl(request.withVelocity(8).withFeedForward(0.5));
  }

  public enum LauncherMode {
    speaker, amp, off;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean getLauncherReady() {
    return launcherReady;
  }

  public void setLauncherMode(LauncherMode mode) {
    this.launcherMode = mode;
  }

  public void stopShooter() {
    this.launcherMode = LauncherMode.off;
    runLauncher();
  }


  @Override
  public void periodic() {
      //if certain toggle is pressed, update LauncherMode
      if (joysticks.syncLauncherMode()) { //need joystick file?
        if (launcherMode != LauncherMode.off) {
          setLauncherMode(LauncherMode.off);
        }
        else {
          setLauncherMode(modes[launcherModeIndex]);
        }
      }
      //if button is pressed to set LauncherScrollPressed to false, set LauncherScrollPressed to false
      if (joysticks.getNotPressed()) {
        launcherScrollPressed = false;
      }
      //if joystick is scrolled right, increase LauncherModeIndex by 1
      if (joysticks.getLauncherScrollRight() && !launcherScrollPressed){
        if (launcherModeIndex < modes.length - 1){
          launcherModeIndex ++;
        }
        launcherScrollPressed = true;
      }
      //if joystick is scrolled left, decrease LauncherModeIndex by 1
      if (joysticks.getLauncherScrollLeft() && !launcherScrollPressed){
        if (launcherModeIndex > 0){
          launcherModeIndex --;
        }
        launcherScrollPressed = true;
      }
      //finally call runLauncher
      runLauncher();
  }

    //runLauncher
    public void runLauncher() {
      //if LauncherMode is default, set predicted velocity values for main and roller
      if (launcherMode == LauncherMode.speaker) {
        setVelocityMain = 0; //TBD, value between -1 and 1
        setVelocityRoller = 0; //TBD
      }
      else if (launcherMode == LauncherMode.amp) {
        setVelocityMain = 0; //TBD
        setVelocityRoller = 0; //TBD
      }
      else if (launcherMode == LauncherMode.off) {
        setVelocityMain = 0; //TBD
        setVelocityRoller = 0; //TBD
      }
      //unless predicted velocity values are 0, set velocity onto the two actual Talons
      if (setVelocityMain == 0){
        mainTalon.set(0);
      }
      else {
        mainTalon.set(setVelocityMain);
      }
      //set Launcher to ready if: absolute difference between the predicted velocity and the actual velocity is less than deadzone for both main and roller and either velocity isn't 0
      if (setVelocityMain == 0){
        rollerTalon.set(0);
      }
      else {
        rollerTalon.set(setVelocityMain);
      }

      launcherReady = Math.abs(Math.abs(setVelocityMain) - Math.abs(mainTalon.getVelocity().getValueAsDouble())) <= Constants.kLauncherTolerance &&
                      Math.abs(Math.abs(setVelocityRoller) - Math.abs(rollerTalon.getVelocity().getValueAsDouble())) <= Constants.kLauncherTolerance && 
                      setVelocityMain != 0 && 
                      setVelocityRoller != 0;
    }

>>>>>>> 9f0f5b5 (launcher subsystem draft with constants)
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
