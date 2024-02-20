package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
  }

  //AutoLock System
  public double getAngleSpeakerDEGS(){
    Pose2d currentRobotPose = SwerveSubsystem.getRobotPose();

    double desiredAngle = currentRobotPose.minus(CompConstants.BLUE_SPEAKER_POSE).getTranslation().getAngle().getDegrees();

    double xRobot = currentRobotPose.getX();
    double yRobot = currentRobotPose.getY();
    double robotDirection = currentRobotPose.getRotation().getDegrees();

    double distanceFromSpeaker = Math.sqrt(Math.pow(AimingConstants.SPEAKER_X_COORDINATE + xRobot, 2) + Math.pow(AimingConstants.SPEAKER_Y_COORDINATE + yRobot, 2)); //distance formula
    double angleRADS = Math.atan((AimingConstants.SPEAKER_HEIGHT - ) / distanceFromSpeaker); //triangle shenanigans (source: trust me bro - Pythagoras)
    double angleDEGS = Math.toDegrees(angleRADS);
    return angleDEGS;
  }

  // Get distance between robot & speaker
  public double getDistanceToSpeaker(){
    return 0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredRotation = getAngleSpeakerDEGS() - AimingConstants.WRIST_PIVOT_ANGLE_OFFSET;

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
