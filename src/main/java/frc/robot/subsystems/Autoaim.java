package frc.robot.subsystems;

import java.sql.Driver;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AimingConstants;

public class Autoaim {

    // only chance this ever works is if pose estimation works as it uses field coords
    private static double neededlauncherAngleRadians = 0.0;
    private static double neededRobotYaw = 0.0;
    SwerveSubsystem swerveSubsystem;
    AimingSubsystem aimingSubsystem;

    private static double prevSpeedX = 0.0;
    private static double prevSpeedY = 0.0;

    private static double lastTime = 0.0;
    private static double currentTime = 0.0;

    public static boolean accountForVelocity = true;

    public static void update(){
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTime;

        prevSpeedX = SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond;
        prevSpeedY = SwerveSubsystem.getChassisSpeeds().vyMetersPerSecond;
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(SwerveSubsystem.getChassisSpeeds(), SwerveSubsystem.getRobotPose().getRotation());
        
        // predict future robot location
        double accelX = (speeds.vxMetersPerSecond - prevSpeedX) / deltaTime;
        double accelY = (speeds.vyMetersPerSecond - prevSpeedY) / deltaTime;

        double velX = speeds.vxMetersPerSecond + accelX * AimingConstants.AUTOAIM_TIME_LOOKAHEAD;
        double velY = speeds.vyMetersPerSecond + accelY * AimingConstants.AUTOAIM_TIME_LOOKAHEAD;

        double robotX = SwerveSubsystem.getRobotPose().getX() + velX * AimingConstants.AUTOAIM_TIME_LOOKAHEAD;
        double robotY = SwerveSubsystem.getRobotPose().getY() + velY * AimingConstants.AUTOAIM_TIME_LOOKAHEAD;


        // get field launcher pivot location
        double launcherPivotX = robotX - SwerveSubsystem.getRobotPose().getRotation().getCos() * Units.inchesToMeters(11.25);
        double launcherPivotY = robotY - SwerveSubsystem.getRobotPose().getRotation().getSin() * Units.inchesToMeters(11.25);
        double launcherPivotZ = Units.inchesToMeters(25.055);


        double xDistance = 0.0;
        double yDistance = 0.0;
        double zDistance = 0.0;

        if (!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red){
            xDistance = AimingConstants.BLUE_SPEAKER_POS.getX() - launcherPivotX;
            yDistance = AimingConstants.BLUE_SPEAKER_POS.getY() - launcherPivotY;
            zDistance = AimingConstants.BLUE_SPEAKER_POS.getZ() - launcherPivotZ;
        }else{
            xDistance = AimingConstants.RED_SPEAKER_POS.getX() - launcherPivotX;
            yDistance = AimingConstants.RED_SPEAKER_POS.getY() - launcherPivotY;
            zDistance = AimingConstants.RED_SPEAKER_POS.getZ() - launcherPivotZ;
        }

        
        double distance2D = Math.sqrt(xDistance * xDistance + yDistance * yDistance);        

        double speakerApproachSpeed = accountForVelocity ? speeds.vxMetersPerSecond * (Math.abs(xDistance)/distance2D) + speeds.vyMetersPerSecond * (Math.abs(yDistance)/distance2D) : 0.0;
        
        neededlauncherAngleRadians = searchForLaunchAngle(distance2D, zDistance, speakerApproachSpeed);

        double xSpeed = accountForVelocity ? speeds.vxMetersPerSecond : 0.0;
        double ySpeed = accountForVelocity ? speeds.vyMetersPerSecond : 0.0;

        neededRobotYaw = searchForYaw(xDistance, yDistance, xSpeed, ySpeed);

        SmartDashboard.putNumber("Launcher Angle Deg", neededlauncherAngleRadians * 180.0 / Math.PI);
        SmartDashboard.putNumber("Robot Yaw", neededRobotYaw * 180.0 / Math.PI);


        if (xDistance < 0 && yDistance < 0) neededRobotYaw -= Math.PI;
        else if (xDistance < 0 && yDistance >= 0) neededRobotYaw += Math.PI;
    }    

    public static double getNeededLaunchAngle(){
        return neededlauncherAngleRadians;
    }

    public static double getNeededRobotYaw(){
        return neededRobotYaw;
    }

    private static double launcherAngleEquation(double xDist2D, double yDist2D, double speakerApproachSpeed, double angleGuess){
        double time = ((xDist2D + AimingConstants.WRIST_D1 * Math.cos(-angleGuess + AimingConstants.WRIST_BEND_ANGLE) + AimingConstants.WRIST_D2 * Math.cos(angleGuess)) / (AimingConstants.NOTE_EXIT_SPEED * Math.cos(angleGuess) + speakerApproachSpeed));
        return -(9.807/2.0) * Math.pow(time, 2) + AimingConstants.NOTE_EXIT_SPEED * Math.sin(angleGuess) * time + AimingConstants.WRIST_D1 * Math.sin(-angleGuess + AimingConstants.WRIST_BEND_ANGLE) - AimingConstants.WRIST_D2 * Math.sin(angleGuess) - yDist2D;
    }

    private static double robotYawEquation(double xDist3D, double yDist3D, double xVel, double yVel, double yawGuess)
    {
        return xDist3D * (yVel - Math.sin(yawGuess) * Math.cos(neededlauncherAngleRadians) * AimingConstants.NOTE_EXIT_SPEED) / (xVel - Math.cos(yawGuess) * Math.cos(neededlauncherAngleRadians) * AimingConstants.NOTE_EXIT_SPEED) - yDist3D;
    }
    

    private static double searchForLaunchAngle(double xDist2D, double yDist2D, double speakerApproachSpeed){
        double step = 0.01;

        double start = 0.0; // zero because anything less would be pointing down
        double end = AimingConstants.REST_LAUNCH_ANGLE * Math.PI / 180.0; // max launch angle
        double sign = Math.signum(launcherAngleEquation(xDist2D, yDist2D, speakerApproachSpeed, start));


        for (double guess = start; guess <= end; guess += step){
            if (Math.signum(launcherAngleEquation(xDist2D, yDist2D, speakerApproachSpeed, guess)) != sign){
                return guess;
            }
        }

        return end;
    }

    private static double searchForYaw(double xDist3D, double yDist3D, double xVel, double yVel){
        double step = 0.01;

        double start = -90 * Math.PI / 180.0;
        double end = 90 * Math.PI / 180.0;
        double sign = Math.signum(robotYawEquation(xDist3D, yDist3D, xVel, yVel, start));

        for (double i = start; i <= end; i += step){
            double guess = robotYawEquation(xDist3D, yDist3D, xVel, yVel, i);
            if (Math.signum(guess) != sign){ // tbh don't know why im using sign atp
                if (Math.abs(guess) < 2.0 * Math.PI / 180.0){
                    return guess;
                }
            }
        }


        return 0.0;
    }

}
