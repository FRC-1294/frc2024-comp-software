// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.JoystickConstants;

// Input Class For Joystick/Controller Input Functions
public class Input {
  private Input() {
    throw new IllegalStateException("Input Class");
  }

  private static final Joystick mRotJoystick = new Joystick(JoystickConstants.ROT_JOYSTICK_PORT);
  private static final Joystick mTransJoystick = new Joystick(JoystickConstants.TRANS_JOY_PORT);
  public static final XboxController mXBox = new XboxController(JoystickConstants.XBOX_PORT);

  private static final XboxController mControllerXbox =
    new XboxController(JoystickConstants.XBOX_PORT);


  public static final int DPADUP = 0;
  public static final int DPADRIGHT = 90;
  public static final int DPADDOWN = 180;
  public static final int DPADLEFT = 270;


  public static boolean resetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean resetOdo() {
    return mTransJoystick.getRawButton(3);
  }

  public static double getJoystickX() {
    return mTransJoystick.getX();
  }

  public static double getJoystickY() {
    return mTransJoystick.getY();
  }

  public static boolean getA() {
    return mControllerXbox.getAButtonPressed();
  }

  public static boolean getB() {
    return mControllerXbox.getBButtonPressed();
  }

  public static boolean getX() {
    return mControllerXbox.getXButtonPressed();
  }

  public static boolean getY() {
    return mControllerXbox.getYButtonPressed();
  }

  public static boolean getLeftBumper() {
    return mControllerXbox.getLeftBumper();
  }

  public static boolean getRightBumper() {
    return mControllerXbox.getRightBumperPressed();
  }

  public static double getLeftTrigger() {
    return mControllerXbox.getLeftTriggerAxis();
  }

  public static double getRightTrigger() {
    return mControllerXbox.getRightTriggerAxis();
  }

  public static double getLeftStickY() {
    return -mControllerXbox.getLeftY();
  }

  public static double getRightStickY() {
    return -mControllerXbox.getRightY();
  }

  public static double getRot() {
    return mRotJoystick.getX();
  }

  public static boolean getResetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean getPrecisionToggle() {
    return mTransJoystick.getTriggerPressed();
  }

  public static boolean getIncPID() {
    return mRotJoystick.getRawButton(5);
  }

  public static boolean getDecPID() {
    return mRotJoystick.getRawButton(4);
  }

  public static boolean togglePIDTuning() {
    return mRotJoystick.getTriggerReleased();
  }

  public static boolean getIntake() {
    return mXBox.getLeftBumper();
  }

  public static boolean overrideIntakeBeamBreak() {
    return mXBox.getRightStickButton();
  }
}
