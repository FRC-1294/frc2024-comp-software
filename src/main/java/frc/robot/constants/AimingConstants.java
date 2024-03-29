// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.Util.PIDParameters;
import frc.robot.subsystems.AimingSubsystem.AimingMotorMode;

// AimingConstants class is used to store all IDs & constants of the Elevator & Wrist components
public class AimingConstants {

    // TODO: Find the angle measures of these setpoints
    // Issue#262 has been created for the same

    // Setpoints
    public enum AimState {
        STOW(MIN_ELEVATOR_DIST_METERS, MIN_WRIST_ROTATION_DEG),
        SPEAKER(MIN_ELEVATOR_DIST_METERS, MIN_WRIST_ROTATION_DEG),
        AMP(MIN_ELEVATOR_DIST_METERS, MIN_WRIST_ROTATION_DEG),
        HANDOFF(MIN_ELEVATOR_DIST_METERS, MIN_WRIST_ROTATION_DEG),
        CLIMB(MIN_ELEVATOR_DIST_METERS, MIN_WRIST_ROTATION_DEG);

        public final double elevatorDistIn;
        public final double wristAngleDeg;
    
        AimState(double dist, double rot) {
          elevatorDistIn = dist;
          wristAngleDeg = rot;
        }
    }
    
    // Teleop Constant
    public static final double MAX_ELEVATOR_TELEOP_INCREMENT = 0.05;
    public static final double MAX_WRIST_TELEOP_INCREMENT = 0.15;
    
    
    // PID Constants
    public static final PIDParameters mElevatorPIDConstants = new PIDParameters(3, 0, 0);
    public static final PIDParameters mWristPIDConstants = new PIDParameters(0.03, 0, 0, 0, 0);
    public static final double WRIST_KG = 0.03;
    public static final double ELEVATOR_FEEDFORWARD_CONSTANT = 0.05;
    public static final double SPARK_THROUGHBORE_GEAR_RATIO = 1;


    public static final AimingMotorMode INITIAL_MOTOR_MODE = AimingMotorMode.BRAKE;

    // TODO: Calculate the ratio for the competition robot
    // Issue #281 has been created for the same. Link to the issue: https://github.com/FRC-1294/frc2024/issues/281
    public static final double ELEVATOR_ROTATIONS_TO_METERS = 0.00626;
	public static final double ELEVATOR_TOLERANCE_IN = 0.01;
    public static final double WRIST_TOLERANCE_DEG = 2;

    // Soft Limits
    public static final double MAX_ELEVATOR_DIST_METERS = 0.5; //TBD
    public static final double MIN_ELEVATOR_DIST_METERS = 0.0; //TBD
    public static final double MAX_ELEVATOR_EXTENSION_VELOCITY = 0.1; //TBD

    public static final double MAX_WRIST_ROTATION = 110;
    public static final double MIN_WRIST_ROTATION_DEG = 0;
    public static final double MAX_WRIST_ROTATION_VELOCITY = 0;

    public static final double MAX_WRIST_PID_CONTRIBUTION = 0.15;
    public static final double MAX_ELEVATOR_PID_CONTRIBUTION = 0.15;

    // ID's
    public static final int LEFT_ELEVATOR_SPARK_ID = 33; //Done
    public static final int RIGHT_ELEVATOR_SPARK_ID = 32; //Done

    public static final int LEFT_WRIST_SPARK_ID = 35; //Done
    public static final int RIGHT_WRIST_SPARK_ID = 34; //Done

    public static final int WRIST_THROUGHBORE_ENCODER_ID = 0;
    public static final double WRIST_THROUGHBORE_GEAR_RATIO = 1;
    public static final double WRIST_THROUGHBORE_ENCODER_OFFSET = 289.4;
    public static final double COG_OFFSET = 27.9;


    // If false, then motors are physically inverted
    public static final boolean ELEVATOR_LEFT_IS_NORMAL = true;
    public static final boolean WRIST_LEFT_IS_NORMAL = false;

    public static final int CONNECTION_THRESH_HZ = 945;
}
