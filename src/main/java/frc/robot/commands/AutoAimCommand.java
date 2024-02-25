package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.CompConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Input;

public class AutoAimCommand extends Command {
  private final AimingSubsystem mAimingSubsystem;
  private final SwerveSubsystem mSwerveSubsystem;
  private final PIDController mRotPID = new PIDController(0.1, 0, 0);

  // Use the Arm from last year as inspiration
  public AutoAimCommand(AimingSubsystem aimingSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    mSwerveSubsystem = swerveSubsystem;
    mAimingSubsystem = aimingSubsystem;

    mRotPID.setTolerance(5);
    addRequirements(aimingSubsystem);
    addRequirements(swerveSubsystem);

    //mSwerveSubsystem = swerveSubsystem;
    //addRequirements(swerveSubsystem);
  }

  //TODO - Create a Pose2D object for the Speaker and do poseRobot.minus(poseSpeaker) instead of distance formula
  //TODO - PoseEstimation does not change whether on Red Alliance or Blue alliance sp use getAlliance() from DriverStation and change code depending

  //AutoLock System
  public double getVerticalAngleSpeakerDEGS(){
    Pose2d currentRobotPose2D = SwerveSubsystem.getRobotPose();
    // Pose3d currentRobotPose3D = new Pose3d(currentRobotPose2D.getX(), currentRobotPose2D.getY(), 0.0, currentRobotPose2D.getRotation());

    // double distanceFromSpeaker3D = currentRobotPose3D.minus(CompConstants.BLUE_SPEAKER_POSE_3D);
    // double distanceFromSpeaker2D = currentRobotPose2D.minus(CompConstants.BLUE_SPEAKER_POSE_2D);
    // double verticalAngle = Math.toDegrees(Math.acos(distanceFromSpeaker2D / distanceFromSpeaker3D));
    return 0;
    }

  // Get distance between robot & speaker
  public double getHorizontalAngleSpeakerDEGS(){
    Pose2d currentRobotPose = SwerveSubsystem.getRobotPose();

    //Equation: angle needed to add = 180 - current angle - angle relative to top of field
    //Assumptions: current angle < 180 ???

    double currentAngle = currentRobotPose.getRotation().getDegrees(); //THETA C
    double cornerSpeakerAngle = Math.atan((AimingConstants.SPEAKER_Y_COORDINATE - currentRobotPose.getY()) / (currentRobotPose.getX() - AimingConstants.SPEAKER_X_COORDINATE)); //THETA F
    double addedAngle = 180 - currentAngle - cornerSpeakerAngle; //THETA A
    return addedAngle;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredVerticalRotation = getVerticalAngleSpeakerDEGS() - AimingConstants.WRIST_PIVOT_ANGLE_OFFSET;
    double desiredHorizontalRotation = getHorizontalAngleSpeakerDEGS();

    mAimingSubsystem.setDesiredElevatorDistance(0);
    mAimingSubsystem.setDesiredWristRotation(desiredVerticalRotation);

    double rotSpeed = mRotPID.calculate(desiredHorizontalRotation, SwerveSubsystem.getHeading());
    mSwerveSubsystem.setChassisSpeed(0, 0, rotSpeed, true, false);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mAimingSubsystem.atSetpoints() && mRotPID.atSetpoint();
    // return false;
  }
}
