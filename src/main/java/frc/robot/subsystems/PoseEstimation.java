// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Input;
import frc.robot.constants.CompConstants;
import frc.robot.constants.VisionConstants;



public class PoseEstimation extends SubsystemBase {
  
  private static PhotonCameras mCameras = new PhotonCameras();

  private static SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(SwerveSubsystem.getKinematics(),
        SwerveSubsystem.getRotation2d(), SwerveSubsystem.getModulePositions(), SwerveSubsystem.getRobotPose(),
        VisionConstants.STATE_STD_DEVS, VisionConstants.VISION_MEASUREMENTS_STD_DEVS);

  private static Field2d mField = new Field2d();

  public PoseEstimation() {
    SmartDashboard.putData("Field", mField);
  }

  @Override
  public void periodic() {
    mPoseEstimator.update(SwerveSubsystem.getRotation2d(), SwerveSubsystem.getModulePositions());
    updateVision();

    Pose2d pose = mPoseEstimator.getEstimatedPosition();
    mField.setRobotPose(pose);
    SmartDashboard.putData("Field", mField);

    if (Input.resetOdo()) {
      resetPose();
    }

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("PoseEst X", pose.getX());
      SmartDashboard.putNumber("PoseEst Y", pose.getY());
      SmartDashboard.putNumber("PoseEst Rot", pose.getRotation().getDegrees());
    }
  }
  
  private void updateVision() {
    boolean updatedFront = updateVisionBack();
    boolean updatedBack = updateVisionFront();

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putBoolean("Updating Front", updatedFront);
      SmartDashboard.putBoolean("Updating Back", updatedBack);
    }
  }

  // Get Pose From Back Camera
  private boolean updateVisionBack() {
    Optional<EstimatedRobotPose> pose =
        mCameras.getEstimatedGlobalPoseBack(mPoseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
            camPose.timestampSeconds);
        return true;
      }
    }
    return false;
  }

  // Get Pose From Front Camera
  private boolean updateVisionFront() {
    Optional<EstimatedRobotPose> pose =
        mCameras.getEstimatedGlobalPoseFront(mPoseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
            camPose.timestampSeconds);
        return true;
      }
    }

    return false;
  }

  private boolean isValidPose(EstimatedRobotPose pose) {
    List<PhotonTrackedTarget> targets = pose.targetsUsed;
    if (targets.size() == 1) {
      return targets.get(0).getPoseAmbiguity() < VisionConstants.SINGLE_TAG_AMBIGUITY_THRESH;
    }

    return true;
  }
  
  public static Pose2d getRobotPose() {
    return mPoseEstimator.getEstimatedPosition();

  }

  public static void resetPose() {
    mPoseEstimator.resetPosition(SwerveSubsystem.getRotation2d(), SwerveSubsystem.getModulePositions(),
        new Pose2d());
  }

  public static void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(SwerveSubsystem.getRotation2d(), SwerveSubsystem.getModulePositions(), pose);
  }

  public static void resetGyro(){
        mPoseEstimator.resetPosition(SwerveSubsystem.getRotation2d(), SwerveSubsystem.getModulePositions(),
        new Pose2d(PoseEstimation.getRobotPose().getX(),PoseEstimation.getRobotPose().getY(), new Rotation2d()));
  }
}