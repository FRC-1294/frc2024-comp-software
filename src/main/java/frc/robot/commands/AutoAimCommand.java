package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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

  // Use the Arm from last year as inspiration
  public AutoAimCommand(AimingSubsystem aimingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    mAimingSubsystem = aimingSubsystem;
    addRequirements(aimingSubsystem);

    //mSwerveSubsystem = swerveSubsystem;
    //addRequirements(swerveSubsystem);
  }

  //TODO - Create a Pose2D object for the Speaker and do poseRobot.minus(poseSpeaker) instead of distance formula
  //TODO - PoseEstimation does not change whether on Red Alliance or Blue alliance sp use getAlliance() from DriverStation and change code depending

  //AutoLock System
  public double getVerticalAngleSpeakerDEGS(){
    Pose2d currentRobotPose = SwerveSubsystem.getRobotPose();
    var alliance = DriverStation.getAlliance();

    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
      //double desiredAngle = currentRobotPose.minus(CompConstants.BLUE_SPEAKER_POSE).getTranslation().getAngle().getDegrees(); ???
      double xRobot = currentRobotPose.getX();
      double yRobot = currentRobotPose.getY();

      double distanceFromSpeaker = Math.sqrt(Math.pow(xRobot - AimingConstants.SPEAKER_X_COORDINATE, 2) + Math.pow(AimingConstants.SPEAKER_Y_COORDINATE - yRobot, 2)); //distance formula
      double verticalAngle = Math.toDegrees(Math.atan((AimingConstants.SPEAKER_HEIGHT_TO_ROBOT) / distanceFromSpeaker)); //triangle shenanigans (source: trust me bro - Pythagoras)
      return verticalAngle;
    }
    
    return 0;
  }

  // Get distance between robot & speaker
  public double getHorizontalAngleSpeakerDEGS(){
    Pose2d currentRobotPose = SwerveSubsystem.getRobotPose();

    double xRobot = currentRobotPose.getX();
    double yRobot = currentRobotPose.getY();

    //Equation: angle needed to add = 180 - current angle - angle relative to top of field
    //Assumptions: current angle < 180 ???
    double currentAngle = currentRobotPose.getRotation().getDegrees(); //THETA C
    double cornerSpeakerAngle = Math.atan((AimingConstants.SPEAKER_Y_COORDINATE - yRobot) / (xRobot - AimingConstants.SPEAKER_X_COORDINATE)); //THETA F
    double addedAngle = 180 - currentAngle - cornerSpeakerAngle;//THETA A
    return addedAngle;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredRotation = getVerticalAngleSpeakerDEGS() - AimingConstants.WRIST_PIVOT_ANGLE_OFFSET;

    mAimingSubsystem.setDesiredElevatorDistance(0);
    mAimingSubsystem.setDesiredWristRotation(desiredRotation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mAimingSubsystem.atSetpoints();
    // return false;
  }
}
