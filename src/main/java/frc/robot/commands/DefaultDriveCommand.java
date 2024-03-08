// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends Command {

  private final SwerveSubsystem mSwerve;
  private boolean mIsPrecisionToggle = false;
  private final PIDController mNotePID = new PIDController(5, 0, 0.1);
  private final PIDController mSpeakerAlignPID = new PIDController(5, 0, 0.1);

  public DefaultDriveCommand(SwerveSubsystem swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
    mNotePID.setTolerance(2);
    mSpeakerAlignPID.setTolerance(2);
    mSpeakerAlignPID.enableContinuousInput(0, 360);
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

    if (Input.alignSpeaker()){
      rot = mSpeakerAlignPID.calculate(SwerveSubsystem.getRobotPose().getRotation().getDegrees(), getRotationToSpeakerDegrees());
    }

    if (Input.resetOdo()) {
      mSwerve.resetRobotPose();
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

    x *= mSwerve.mConfig.TELE_MAX_SPEED_MPS;
    y *= mSwerve.mConfig.TELE_MAX_SPEED_MPS;
    rot *= mSwerve.mConfig.TELE_MAX_ROT_SPEED_RAD_SEC;
    mSwerve.setChassisSpeed(x, y, rot, !Input.getRobotOriented(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getRotationToSpeakerDegrees(){
    Transform2d relativeTrans;
    if (Robot.mAlliance.get() == Alliance.Red){
      relativeTrans = FieldConstants.Red.SPEAKER.getPose().toPose2d().minus(SwerveSubsystem.getRobotPose());
    } else{
      relativeTrans = FieldConstants.Blue.SPEAKER.getPose().toPose2d().minus(SwerveSubsystem.getRobotPose());
    }
    //Use atan2 to account for launching on blue side
    double targAngle = Math.toDegrees(Math.atan2(relativeTrans.getY(), relativeTrans.getX()));
    if (targAngle < 0){
      //This is to convert the range from [-180,180] to [0,360]
      targAngle += 360;
    }
    //add 180 degrees because drivebase 0 is relative to intake, we want it to be relative to launcher
    targAngle += 180;
    if (targAngle>360){
      //Take the remainder since we don't want angles above 360
      targAngle = Math.IEEEremainder(targAngle, 360);
    }
    return targAngle;
  }
}