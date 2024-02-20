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

    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(16.5, 5.5, Rotation2d.fromDegrees(180));
    
}
