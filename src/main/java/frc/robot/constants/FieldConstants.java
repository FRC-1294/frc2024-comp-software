package frc.robot.constants;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public class FieldConstants {
    public static enum Blue {
        SPEAKER(new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));

        private final Pose3d pose;

        Blue(Pose3d pose) {
            this.pose = pose;
        }

        public Pose3d getPose() {
            return pose;
        }
    }

    public static enum Red {
        SPEAKER(new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));

        private final Pose3d pose;

        Red(Pose3d pose) {
            this.pose = pose;
        }

        public Pose3d getPose() {
            return pose;
        }
    }


    public static double getSpeakerDistance(Pose2d robotPose) {
        if (Robot.mAlliance.isPresent() && Robot.mAlliance.get() == Alliance.Red) {
            return Red.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();            
        }
        return Blue.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();

    }
}
