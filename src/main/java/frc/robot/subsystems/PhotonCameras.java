// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public PhotonCameras() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        mPhotonCameraFront = new PhotonCamera(VisionConstants.CAMERA_NAME_FRONT);
        mPhotonCameraBack = new PhotonCamera(VisionConstants.CAMERA_NAME_BACK);

        
        AprilTagFieldLayout fieldLayout =
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
        // This method will be called once per scheduler run
        Optional<EstimatedRobotPose> poseFront = getEstimatedGlobalPoseFront(SwerveSubsystem.getRobotPose());
        Optional<EstimatedRobotPose> poseBack = getEstimatedGlobalPoseBack(SwerveSubsystem.getRobotPose());

        if (poseFront.isPresent() && isValidPose(poseFront.get())) {
            SwerveSubsystem.updateVision(poseFront.get());
        } else if (poseBack.isPresent() && isValidPose(poseBack.get())) {
            SwerveSubsystem.updateVision(poseBack.get());
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
    }

    return true;
    }
}