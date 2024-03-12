// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;
import java.util.Optional;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.Input;
import frc.robot.subsystems.Autoaim;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends Command {

  private final SwerveSubsystem mSwerve;
  private boolean mIsPrecisionToggle = false;
  private final PIDController mNotePID = new PIDController(5, 0, 0.1);
  private final PIDController yawAutoaim = new PIDController(1, 0, 0.1);

  private final PIDController mSpeakerAlignPID = new PIDController(4, 0, 0.02);

  public DefaultDriveCommand(SwerveSubsystem swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
    mNotePID.setTolerance(2);
    yawAutoaim.enableContinuousInput(-180, 180);

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
    boolean isFieldOriented = true;

    if (Input.resetGyro()) {
      mSwerve.resetGyro();
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
    // SmartDashboard.putNumber("tx", mLimelight.getNoteAngle());
    // if (Input.getNoteAlignment() && mLimelight.isDetectionValid()) {
    //     rot = mNotePID.calculate(Units.degreesToRadians(mLimelight.getNoteAngle()));
    //     isFieldOriented = false;
    // }

    // if (Input.doAutoaim()) {
    //   rot = yawAutoaim.calculate(SwerveSubsystem.getRobotPose().getRotation().getDegrees(), Units.radiansToDegrees(Autoaim.getNeededRobotYaw()));
    //   //SmartDashboard.putNumber("Autoaim", rot)
    // }

    mSwerve.setChassisSpeed(x, y, Math.toRadians(rot), true, false);

    if (Input.alignSpeaker()){
      rot = Math.toRadians(
        mSpeakerAlignPID.calculate(
          SwerveSubsystem.getHeading(),
          getRotationToSpeakerDegrees()));
      SmartDashboard.putBoolean("PodiumAligned", mSpeakerAlignPID.atSetpoint());

    }
    SmartDashboard.putBoolean("Precision Toggle", mIsPrecisionToggle);
    mSwerve.setChassisSpeed(x, y, rot, true, false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getRotationToSpeakerDegrees(){
    double targAngle;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
      targAngle = 35.15;
    } else{
      targAngle = 324.85;
    }

    //Use atan2 to account for launching on blue side
    //double targAngle = Math.toDegrees(Math.atan2(relativeTrans.getY(), relativeTrans.getX()));
    // if (targAngle < 0){
    //   //This is to convert the range from [-180,180] to [0,360]
    //   targAngle += 360;
    // }
    //add 180 degrees because drivebase 0 is relative to intake, we want it to be relative to launcher
    // targAngle += 180;
    // if (targAngle>360){
    //   //Take the remainder since we don't want angles above 360
    //   targAngle = Math.IEEEremainder(targAngle, 360);
    // }
    return targAngle;
  }
}