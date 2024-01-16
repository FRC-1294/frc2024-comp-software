// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
