// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private static final XboxController mXboxController2 =
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
    return mXboxController2.getAButtonPressed();
  }

  public static boolean getB() {
    return mXboxController2.getBButtonPressed();
  }

  public static boolean getX() {
    return mXboxController2.getXButtonPressed();
  }

  public static boolean getY() {
    return mXboxController2.getYButtonPressed();
  }

  public static boolean getLeftBumper() {
    return mXboxController2.getLeftBumper();
  }

  public static boolean getRightBumper() {
    return mXboxController2.getRightBumperPressed();
  }

  public static double getLeftTrigger() {
    return mXboxController2.getLeftTriggerAxis();
  }

  public static double getRightTrigger() {
    return mXboxController2.getRightTriggerAxis();
  }

  public static double getLeftStickY() {
    return -mXboxController2.getLeftY();
  }

  public static double getRightStickY() {
    return -mXboxController2.getRightY();
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
}

