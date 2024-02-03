// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameras {
    private final PhotonCamera mPhotonCameraFront;
    private final PhotonCamera mPhotonCameraBack;
    private PhotonPoseEstimator mPhotonPoseEstimatorFront;
    private PhotonPoseEstimator mPhotonPoseEstimatorBack;

    public PhotonCameras() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        mPhotonCameraFront = new PhotonCamera(VisionConstants.CAMERA_NAME_FRONT);
        mPhotonCameraBack = new PhotonCamera(VisionConstants.CAMERA_NAME_BACK);

        
        AprilTagFieldLayout fieldLayout =
                AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
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

}