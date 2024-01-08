// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDConstants;

/** Add your docs here. */
public class LauncherConstants {
    public enum LauncherState{
        READY(1000,0),//TODO
        STATE_1(2000,0),//TODO
        STATE_2(4000,0),//TODO
        HOME(0,0);//TODO
        public final double mFlyWheelRPM;
        public final double mRollerRPM;
        LauncherState(double flyWheelRPM, double rollerRPM){
          mFlyWheelRPM = flyWheelRPM;
          mRollerRPM = rollerRPM;
        }
      }

    public static final double ENCODER_TICKS_PER_ROTATION = 2048;
    public static final double PHYSICAL_MAX_RPM_FALCON = 6380;

    //2048 sensor units/ 100 ms  -> RPM
    public static final double ENCODER_VEL_TO_MOTOR_RPM = 600/ENCODER_TICKS_PER_ROTATION; 
    public static final double MOTOR_TO_LEFT_FLWHEEL_GEAR_RATIO = 1;
    public static final double MOTOR_TO_RIGHT_FLYWHEEL_GEAR_RATIO = 1;

    public static final double ENCODER_VEL_TO_LEFT_FLYWHEEL_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_LEFT_FLWHEEL_GEAR_RATIO;
    public static final double ENCODER_VEL_TO_RIGHT_FLYWHEEL_RPM = ENCODER_VEL_TO_MOTOR_RPM * MOTOR_TO_RIGHT_FLYWHEEL_GEAR_RATIO;
    
    public static final double PHYSICAL_MAX_RPM_LEFT_FLYWHEEL = PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_LEFT_FLWHEEL_GEAR_RATIO;
    public static final double PHYSICAL_MAX_RPM_RIGHT_FLYWHEEL= PHYSICAL_MAX_RPM_FALCON * MOTOR_TO_RIGHT_FLYWHEEL_GEAR_RATIO;

    public static final double LEFT_FLYWHEEL_MAX_ACCEL_ROTPS = 14703.66; //TODO
    public static final double RIGHT_FLYWHEEL_MAX_ACCEL_ROTPS = -1; //TODO

    public static final boolean INVERT_LEFT_FLYWHEEL_MOTOR = false;//TODO
    public static final boolean INVERT_RIGHT_FLYWHEEL_MOTOR = true;//TODO

    public static final double LEFT_FLYWHEEL_TBH_CONSTANT = 0.03; //TODO
    public static final double RIGHT_FLYWHEEL_TBH_CONSTANT = 0; //TODO

    public static final boolean LEFT_FLYWHEEL_SENSOR_PHASE = false; //TODO
    public static final boolean RIGHT_FLYWHEEL_SENSOR_PHASE = false; //TODO

    public static final Translation3d LEFT_FLYWHEEL_LOC_M = new Translation3d(-0.5, 0, 0.4); //TODO
    public static final Translation3d RIGHT_FLYWHEEL_LOC_M = new Translation3d(-0.8,0,0.5); //TODO

    public static final double LEFT_FLYWHEEL_RADIUS_M = Units.inchesToMeters(2); //TODO
    public static final double RIGHT_FLYWHEEL_RADIUS_M = Units.inchesToMeters(2); //TODO

    public static final double LEFT_FLYWHEEL_WEIGHT_KG = 0.150*4 + 0.493*2; //TODO
    public static final double RIGHT_FLYWHEEL_WEIGHT_KG = 0.150*4 + 0.493*2; //TODO

    public static final double FLYWHEEL_BAR_WEIGHT_KG = Units.lbsToKilograms(1.8);

    public static final double LEFT_FLYWHEEL_MOI_CONSTANT = 1.0/2; //TODO
    public static final double RIGHT_FLYWHEEL_MOI_CONSTANT = 1.0/2; //TODO

    public static final double LEFT_FLYWHEEL_MOI_KG_MPS = LEFT_FLYWHEEL_MOI_CONSTANT*LEFT_FLYWHEEL_WEIGHT_KG*Math.pow(LEFT_FLYWHEEL_RADIUS_M,2);
    public static final double RIGHT_FLYWHEEL_MOI_KG_MPS = RIGHT_FLYWHEEL_MOI_CONSTANT*RIGHT_FLYWHEEL_WEIGHT_KG*Math.pow(RIGHT_FLYWHEEL_RADIUS_M,2);

    public static final double LEFT_FLYWHEEL_SP_DEADZONE = 0.05;
    public static final double RIGHT_FLYWHEEL_SP_DEADZONE = 0.05;
    
    public static final int LEFT_FLYWHEEL_TALON_ID = 8; //TODO
    public static final int RIGHT_FLYWHEEL_TALON_ID = 7; //TODO

    
    public static final PIDConstants LEFT_FLYWHEEL_PID = //TODO
    new PIDConstants(1,0,0.0,0,0.12,0);
    public static final PIDConstants RIGHT_FLYWHEEL_PID = //TODO
    new PIDConstants(1,0,0,0,0.12,0);

    public static PIDConstants TUNABLE_RIGHT_FLYWHEEL_PID = //TODO
    new PIDConstants(3,1.0,0.02,0,0.17,0);  
    public static PIDConstants TUNABLE_LEFT_FLYWHEEL_PID = //TODO
    new PIDConstants(3,1.0,0.02,0,0.17,0);}

