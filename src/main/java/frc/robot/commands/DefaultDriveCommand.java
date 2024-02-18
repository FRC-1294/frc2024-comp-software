// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends Command {

  private final SwerveSubsystem mSwerve;
  private final Limelight mLimelight;
  private boolean mIsPrecisionToggle = false;
  PIDController notePID = new PIDController(5, 0, 0.1);
  


  public DefaultDriveCommand(SwerveSubsystem swerve, Limelight limelight) {
    mSwerve = swerve;
    mLimelight = limelight;
    addRequirements(mSwerve);
    addRequirements(mLimelight);
    notePID.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // x, y, and rot are inverted because of the Joystick configurations
    double x = -Input.getJoystickY();
    double y = -Input.getJoystickX();
    double rot = -Input.getRot();

    if (Input.resetGyro()) {
      mSwerve.resetGyro();
    }

    if (Input.resetOdo()) {
      mSwerve.resetOdometry();
    }

    if (Input.getPrecisionToggle()) {
      mIsPrecisionToggle = !mIsPrecisionToggle;
    }

    if (mIsPrecisionToggle) {
      x = x / JoystickConstants.DRIVE_PRECISION_X_DESATURATION;
      y = y / JoystickConstants.DRIVE_PRECISION_Y_DESATURATION;
      rot = rot / JoystickConstants.DRIVE_PRECISION_ROT_DESATURATION;

      x = Math.abs(x) > JoystickConstants.DRIVE_PRECISION_X_DEADZONE ? x : 0.0;
      y = Math.abs(y) > JoystickConstants.DRIVE_PRECISION_Y_DEADZONE ? y : 0.0;
      rot = Math.abs(rot) > JoystickConstants.DRIVE_PRECISION_ROT_DEADZONE ? rot : 0.0;
    } else {
      x = Math.abs(x) > JoystickConstants.DRIVE_REG_X_DEADZONE ? x : 0.0;
      y = Math.abs(y) >  JoystickConstants.DRIVE_REG_Y_DEADZONE ? y : 0.0;
      rot = Math.abs(rot) > JoystickConstants.DRIVE_REG_ROT_DEADZONE ? rot : 0.0;
    }

    x *= SwerveConstants.TELE_MAX_SPEED_MPS;
    y *= SwerveConstants.TELE_MAX_SPEED_MPS;
    rot *= SwerveConstants.TELE_MAX_ROT_SPEED_RAD_SEC;
    boolean isFieldOriented = true;
    SmartDashboard.putNumber("tx", mLimelight.getTX());
    if (Input.getNoteAlignment() && mLimelight.isDetectionValid()) {
        rot = notePID.calculate(Units.degreesToRadians(mLimelight.getTX()));
        isFieldOriented = false;
    }
    mSwerve.setChassisSpeed(x, y, rot, isFieldOriented);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}