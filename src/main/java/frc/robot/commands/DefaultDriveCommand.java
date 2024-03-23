// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JoystickConstants;
import frc.robot.Input;
import frc.robot.subsystems.Autoaim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends Command {

  private final SwerveSubsystem mSwerve;
  private boolean mIsPrecisionToggle = false;
  private final PIDController mNotePID = new PIDController(5, 0, 0.1);
  private final PIDController yawAutoaim = new PIDController(4, 0, 0.02);


  public static final PIDController mSpeakerAlignPID = new PIDController(4, 0, 0.02);

  public DefaultDriveCommand(SwerveSubsystem swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
    mNotePID.setTolerance(2);
    yawAutoaim.enableContinuousInput(-180, 180);

    mSpeakerAlignPID.setTolerance(2);
    mSpeakerAlignPID.enableContinuousInput(0, 360);

    BooleanSupplier getDriveBaseLaunchReady = ()-> mSpeakerAlignPID.atSetpoint() &&
     FieldConstants.getSpeakerDistance()<=AimingConstants.MAX_SHOT_DIST_METERS;
    
    new Trigger(getDriveBaseLaunchReady)
    .onTrue(new InstantCommand(()->{
      Input.turnOnViberator(JoystickConstants.XBOX_RUMBLE_VIGEROUS);
      Input.enableLeftRumble(JoystickConstants.XBOX_RUMBLE_VIGEROUS);}))
    .onFalse(new InstantCommand(()->Input.disableLeftRumble()));
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

    if (Input.doAutoaim()) {
      rot = yawAutoaim.calculate(SwerveSubsystem.getRobotPose().getRotation().getDegrees()-180, Units.radiansToDegrees(Autoaim.getNeededRobotYaw()));
      rot = Math.toRadians(rot);
    }

    if (Input.alignSpeaker()){
      rot = Math.toRadians(
        mSpeakerAlignPID.calculate(
          SwerveSubsystem.getRobotPose().getRotation().getDegrees(),
          getRotationToSpeakerDegrees()));
    }

    SmartDashboard.putBoolean("PodiumAligned", mSpeakerAlignPID.atSetpoint());
    SmartDashboard.putBoolean("Precision Toggle", mIsPrecisionToggle);
    mSwerve.setChassisSpeed(x, y, rot, true, false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static double getRotationToSpeakerDegrees(){
    double targAngle;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double distance = FieldConstants.getSpeakerDistance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
      targAngle = FieldConstants.Red.SPEAKER.getPose().toPose2d().plus(new Transform2d(-1.3,0,new Rotation2d()))
      .minus(SwerveSubsystem.getRobotPose()).getTranslation().getAngle().getDegrees();
      
      targAngle += 2*distance;

    } else{
      // targAngle = FieldConstants.Blue.SPEAKER.getPose().toPose2d().plus(new Transform2d(1.3,0,new Rotation2d()))
      // .minus(SwerveSubsystem.getRobotPose()).getTranslation().getAngle().getDegrees();

      double targerAngle = Math.atan2(SwerveSubsystem.getRobotPose().getY() - FieldConstants.Blue.SPEAKER.getPose().getY(), SwerveSubsystem.getRobotPose().getX() - FieldConstants.Blue.SPEAKER.getPose().getX());

      SmartDashboard.putNumber("targeranlge", Math.toDegrees(targerAngle));
      //targAngle += 2*distance;
      targAngle = Math.toDegrees(targerAngle);
    }

    // Tunable Degree Offset Based on Distance
  
    
    SmartDashboard.putNumber("targAngleSPeaker", targAngle);
    return targAngle;
  }
}