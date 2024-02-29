// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public enum AimState {
    SUBWOOFER(0,0,0,3), //Tolerance TBD
    LINE(0,0,0,0), //Everything TBD
    MIDNOTE(0,0,0,0), //Everything TBD
    WING(0,0,0,0), //Everything TBD
    PROTECTED(0,0,0,0), //Everything TBD
    AMP(0,-1,-1,0), //Everything TBD
    TRAP(0,-1,-1,0,0,0), //Everything TBD
    HANDOFF(0,-1,-1,3), //Tolerance TBD
    CLIMB_UP(0,-1,-1,0,AimingConstants.MAX_ELEVATOR_DIST_METERS-0.2,0.1),
    CLIMB_DOWN(0,-1,-1,0,AimingConstants.MIN_ELEVATOR_DIST_METERS,0.1),
    TRANSITION(-1,-1,-1,-1);

    public final double mWristAngleDegrees;
    public final double mRadialDistanceMeters;
    public final double mElevatorHeightMeters;
    public final double mPositionToleranceMeters;
    public final double mWristToleranceDegrees;
    public final double mElevatorToleranceMeters;

    AimState(double wristAngleDeg, double radialDistanceMeters, double shotToleranceMeters,
     double wristToleranceDegrees, double elevatorHeight, double elevatorTolerance) {
        mWristAngleDegrees = wristAngleDeg;
        mRadialDistanceMeters = radialDistanceMeters;
        mElevatorHeightMeters = elevatorHeight;
        mElevatorToleranceMeters = elevatorTolerance;
        mPositionToleranceMeters = shotToleranceMeters;
        mWristToleranceDegrees = wristAngleDeg;
    }

    AimState(double wristAngleDeg, double radialDistanceMeters, double shotToleranceMeters,
     double wristToleranceDegrees) {
        mWristAngleDegrees = wristAngleDeg;
        mRadialDistanceMeters = radialDistanceMeters;
        mElevatorHeightMeters = 0;
        mElevatorToleranceMeters = 0.03;
        mPositionToleranceMeters = shotToleranceMeters;
        mWristToleranceDegrees = wristAngleDeg;
    }

    private double[] getPolarCoordsFromXY(Pose2d curSwervePose){
        double [] coords = new double[2];
        //return the polar coordinates in the form of (r,theta) from the 
        return coords;
    }

    public boolean withinWristTolerance(double curWristAngle){
        return (Math.abs(curWristAngle-mWristAngleDegrees)<=mWristToleranceDegrees || mWristAngleDegrees == -1);
    }
        
    public boolean withinElevatorTolerance(double curElevatorHeight){
        return (Math.abs(curElevatorHeight-mElevatorHeightMeters)<=mElevatorToleranceMeters || mElevatorHeightMeters == -1);
    }

    public boolean withinSwerveTolerance(Pose2d curServePose){
        return true;
    }
    public boolean withinSwerveYawTolerance(){
        return true;
    }

}
