package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * CompConstants
 */
public class CompConstants {
    private CompConstants() {
        throw new IllegalStateException("Constants Class");
    }
    public static final double LOOP_TIME = 0.02;
    public static final boolean DEBUG_MODE = false;

    public static final boolean PID_TUNE_MODE = false;

    public static final Pose3d BLUE_SPEAKER_POSE_3D = new Pose3d(0, 5.55, 2.05, Rotation3d.fromDegrees(180));
    public static final Pose3d RED_SPEAKER_POSE_3D = new Pose3d(16.5, 5.55, 2.05, Rotation3d.fromDegrees(180));

    public static final Pose3d BLUE_SPEAKER_POSE_2D = new Pose2d(0, 5.55, Rotation3d.fromDegrees(180));
    public static final Pose3d RED_SPEAKER_POSE_2D = new Pose2d(16.5, 5.55, Rotation3d.fromDegrees(180));
}
