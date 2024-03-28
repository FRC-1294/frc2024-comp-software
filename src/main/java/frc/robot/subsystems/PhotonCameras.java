// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameras extends SubsystemBase {
    private final PhotonCamera mPhotonCameraFront;
    private final PhotonCamera mPhotonCameraBack;
    private PhotonPoseEstimator mPhotonPoseEstimatorFront;
    private PhotonPoseEstimator mPhotonPoseEstimatorBack;
    private AprilTagFieldLayout fieldLayout;

    public PhotonCameras() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        mPhotonCameraFront = new PhotonCamera(VisionConstants.CAMERA_NAME_FRONT);
        mPhotonCameraBack = new PhotonCamera(VisionConstants.CAMERA_NAME_BACK);

        
        fieldLayout =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // Create pose estimator
        mPhotonPoseEstimatorFront =
                new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        mPhotonCameraFront, VisionConstants.ROBOT_TO_CAM_VEC_FRONT);
        mPhotonPoseEstimatorFront
                .setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        mPhotonPoseEstimatorBack =
                new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        mPhotonCameraBack, VisionConstants.ROBOT_TO_CAM_VEC_BACK);
        mPhotonPoseEstimatorBack
                .setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);


    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleop()||true) {
            // This method will be called once per scheduler run
            Optional<EstimatedRobotPose> poseFront = getEstimatedGlobalPoseFront(SwerveSubsystem.getRobotPose());
            if (poseFront.isPresent() && isValidPose(poseFront.get())) {
                SwerveSubsystem.updateVision(poseFront.get(), getVisionSTD(poseFront.get()));

            } 
            
            Optional<EstimatedRobotPose> poseBack = getEstimatedGlobalPoseBack(SwerveSubsystem.getRobotPose());
            if (poseBack.isPresent() && isValidPose(poseBack.get())) {
                SwerveSubsystem.updateVision(poseBack.get(), getVisionSTD(poseBack.get()));
                SmartDashboard.putNumber("XPosVision", poseBack.get().estimatedPose.getX());
                SmartDashboard.putNumber("YPosVision", poseBack.get().estimatedPose.getY());
                SmartDashboard.putNumber("RotVision", poseBack.get().estimatedPose.getRotation().toRotation2d().getDegrees());
                SmartDashboard.putNumber("ZPhoton", poseBack.get().estimatedPose.getZ());
            }
        }
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose from the FRONT CAMERA with an estimated pose, the timestamp,
     *         and targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(Pose2d prevEstimatedRobotPose) {
        if (!mPhotonCameraFront.isConnected()) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        mPhotonPoseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
        return mPhotonPoseEstimatorFront.update();
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose from the BACK CAMERA with an estimated pose, the timestamp, and
     *         targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack(Pose2d prevEstimatedRobotPose) {
        if (!mPhotonCameraBack.isConnected()) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        mPhotonPoseEstimatorBack.setReferencePose(prevEstimatedRobotPose);
        return mPhotonPoseEstimatorBack.update();
    }


    private boolean isValidPose(EstimatedRobotPose pose) {
    List<PhotonTrackedTarget> targets = pose.targetsUsed;
    if (targets.size() == 1) {
        return targets.get(0).getPoseAmbiguity() < VisionConstants.SINGLE_TAG_AMBIGUITY_THRESH;
    } else if (targets.size() > 1) {
        return true;
    }

    return false;
    }

    private Matrix<N3,N1> getVisionSTD(EstimatedRobotPose pose) {
        List<PhotonTrackedTarget> targets = pose.targetsUsed;
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget target:targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags ++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose.estimatedPose.getTranslation().toTranslation2d());
        }

        if (numTags == 0) return VisionConstants.SINGLE_TAG_VISION_MEASUREMENTS_STD_DEVS;

        avgDist /= numTags;


        // Remove all readings above 4m on avg
        if ((numTags == 1 && avgDist > 4) || (avgDist>5)) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        // Don't scale by distance for Multi Tag Estimation(Literal Magic)
        if (numTags > 1) {
            return VisionConstants.MULTI_TAG_VISION_MEASUREMENTS_STD_DEVS;
        } 
        // Just Scale By Distance        
        return VisionConstants.SINGLE_TAG_VISION_MEASUREMENTS_STD_DEVS.times(1 + avgDist*avgDist/30);
        
        
    }
}