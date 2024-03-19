// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDParameters;
import frc.robot.subsystems.AimingSubsystem.AimingMotorMode;

// AimingConstants class is used to store all IDs & constants of the Elevator & Wrist components
public class AimingConstants {
    
    // Teleop Constant
    public static final double MAX_ELEVATOR_TELEOP_INCREMENT = 0.005;
    public static final double MAX_WRIST_TELEOP_INCREMENT = 0.15;
    
    
    // PID Constants
    public static final PIDParameters mElevatorPIDConstants = new PIDParameters(3, 0, 0);
    public static final PIDParameters mWristPIDConstants = new PIDParameters(0.0075, 0, 0.0, 0, 0);
    public static final double WRIST_KG = 0.043;
    public static final double ELEVATOR_FEEDFORWARD_CONSTANT = 0.05;
    public static final double SPARK_THROUGHBORE_GEAR_RATIO = 1;


    public static final AimingMotorMode INITIAL_MOTOR_MODE = AimingMotorMode.BRAKE;

    // TODO: Calculate the ratio for the competition robot
    // Issue #281 has been created for the same. Link to the issue: https://github.com/FRC-1294/frc2024/issues/281
    public static final double ELEVATOR_ROTATIONS_TO_METERS = 0.00626;
	public static final double ELEVATOR_TOLERANCE_IN = 0.01;
    public static final double WRIST_TOLERANCE_DEG = 2;

    // Soft Limits
    public static final double MAX_ELEVATOR_DIST_METERS = 0.65; //TBD
    public static final double MIN_ELEVATOR_DIST_METERS = 0; //TBD
    public static final double MAX_ELEVATOR_EXTENSION_VELOCITY = 0.1; //TBD

    public static final double MAX_WRIST_ROTATION = 110;
    public static final double MIN_WRIST_ROTATION_DEG = 0;
    public static final double MAX_WRIST_ROTATION_VELOCITY = 0;

    public static final double MAX_WRIST_PID_CONTRIBUTION = 0.6;
    public static final double MAX_ELEVATOR_PID_CONTRIBUTION = 0.3;


    // inaccurate values
    public static final double REST_LAUNCH_ANGLE = 53.429; // rest is when wrist rotation is zero degrees
    public static final double NOTE_EXIT_SPEED = 26.26;
    public static final double AREA1 = (12.0/39.37) * (2.0/39.37) + (Math.PI * Math.pow(1/39.37, 2));
    public static final double AREA2 = Math.PI * (.3556*.3556) - Math.PI * (.254*.254);
    public static final double FLUID_DENSITY = 1.293;
    public static final double MASS = 0.2353010419;

    public static final double DRAG_COEFFICIENT = 0.0;
    public static final double WRIST_D1 = .1778;
    public static final double WRIST_D2 = .00;
    public static final double WRIST_BEND_ANGLE = Units.degreesToRadians(50.429);
    public static final Pose3d BLUE_SPEAKER_POS = new Pose3d(.4, 5.5, 2.05, new Rotation3d());
    public static final Pose3d RED_SPEAKER_POS = new Pose3d(16.14, 5.5, 2.05, new Rotation3d());
    public static final double AUTOAIM_TIME_LOOKAHEAD = 0.0;

    // ID's
    public static final int LEFT_ELEVATOR_SPARK_ID = 33; //Done
    public static final int RIGHT_ELEVATOR_SPARK_ID = 32; //Done

    public static final int LEFT_WRIST_SPARK_ID = 35; //Done
    public static final int RIGHT_WRIST_SPARK_ID = 34; //Done

    public static final int WRIST_THROUGHBORE_ENCODER_ID = 0;
    public static final double WRIST_THROUGHBORE_GEAR_RATIO = 1;
    public static final double WRIST_THROUGHBORE_ENCODER_OFFSET = 298.2-177.44-0.6+0.6+8.4;
    public static final double COG_OFFSET = 27.2;


    // If false, then motors are physically inverted
    public static final boolean ELEVATOR_LEFT_IS_NORMAL = true;
    public static final boolean WRIST_LEFT_IS_NORMAL = true;

    public static final int CONNECTION_THRESH_HZ = 945;

    public static final InterpolatingDoubleTreeMap AIM_MAP = new InterpolatingDoubleTreeMap();

    public static void populate_aim_map(){
        AimingConstants.AIM_MAP.put(AimState.MIDNOTE.mWristAngleDegrees, AimState.MIDNOTE.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.SUBWOOFER.mWristAngleDegrees, AimState.SUBWOOFER.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.LINE.mWristAngleDegrees, AimState.LINE.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.WING.mWristAngleDegrees, AimState.WING.mRadialDistanceMeters);
    }
}
