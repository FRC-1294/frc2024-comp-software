// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.swerve.SwerveModuleAbstract;
import frc.robot.Util.PIDParameters;

/** Add your docs here. */
public abstract class SwerveConfig {
        protected SwerveConfig() {
                config();
        
                //Abstract class cannot be initialized
                // Swerve Module Locations
                FRONT_LEFT_COORDS_METERS =
                        new Translation2d(TRACK_LENGTH_METERS / 2,
                                TRACK_WIDTH_METERS / 2);
                FRONT_RIGHT_COORDS_METERS =
                        new Translation2d(TRACK_LENGTH_METERS / 2,
                                                -TRACK_WIDTH_METERS / 2);
                BACK_LEFT_COORDS_METERS =
                        new Translation2d(-TRACK_LENGTH_METERS / 2,
                                                TRACK_WIDTH_METERS / 2);
                BACK_RIGHT_COORDS_METERS =
                        new Translation2d(-TRACK_LENGTH_METERS / 2,
                                                -TRACK_WIDTH_METERS / 2);

                SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        FRONT_LEFT_COORDS_METERS, FRONT_RIGHT_COORDS_METERS,
                        BACK_LEFT_COORDS_METERS, BACK_RIGHT_COORDS_METERS);

                PIGEON = new Pigeon2(PIGEON_ID, "DriveMotors");

                WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
                TRANS_RPM_TO_MPS =
                (TRANS_GEAR_RATIO_ROT * WHEEL_CIRCUMFERENCE_METERS) / 60;

                FRONT_LEFT_TRANS_ID = 1;
                FRONT_LEFT_ROT_ID = 8;
                FRONT_LEFT_ROT_ENC_ID = 22;

                FRONT_RIGHT_TRANS_ID = 7;
                FRONT_RIGHT_ROT_ID = 3;
                FRONT_RIGHT_ROT_ENC_ID = 23;

                BACK_LEFT_TRANS_ID = 5;
                BACK_LEFT_ROT_ID = 4;
                BACK_LEFT_ROT_ENC_ID = 21;

                BACK_RIGHT_TRANS_ID = 9;
                BACK_RIGHT_ROT_ID = 6;
                BACK_RIGHT_ROT_ENC_ID = 20;


                SWERVE_MODULE_PIDs[0] = FRONT_LEFT_ROT_PID;
                SWERVE_MODULE_PIDs[1] = FRONT_RIGHT_ROT_PID;
                SWERVE_MODULE_PIDs[2] = BACK_LEFT_ROT_PID;
                SWERVE_MODULE_PIDs[3] = BACK_RIGHT_ROT_PID;

                initializeSwerveModules();

                SWERVE_MODULES[0] = FRONT_LEFT_MODULE;
                SWERVE_MODULES[1] = FRONT_RIGHT_MODULE;
                SWERVE_MODULES[2] = BACK_LEFT_MODULE;
                SWERVE_MODULES[3] = BACK_RIGHT_MODULE;


                
        }

        public abstract void config();

        public abstract void initializeSwerveModules();

        public int NUM_MODULES = 4;
        //Since Java does not support abstract FIELDS, this is our only solution apart from requiring everything within the constructor
        public Pigeon2 PIGEON = null;
        public double TRANS_GEAR_RATIO_ROT;
        public double REL_ENC_GEAR_RATIO_ROT = 1 / 12.8;
        public double ABS_ENC_GEAR_RATIO_ROT = -1;
      
        // Conversion Factors
        public double WHEEL_DIAMETER_METERS = -1;
        public double WHEEL_CIRCUMFERENCE_METERS = -1;
        public double TRANS_RPM_TO_MPS = -1;
      
        // Physical Max
        public double PHYSICAL_MAX_SPEED_MPS = -1;
      
        public double TELE_MAX_SPEED_MPS = -1;
        public double TELE_MAX_ROT_SPEED_RAD_SEC = -1;
      
        public double TELE_MAX_ACC_MPS2 = -1;
        public double TELE_MAX_ROT_ACC_RAD_SEC2 = -1;

        public double TRACK_WIDTH_METERS = -1;
        public double TRACK_LENGTH_METERS = -1;
        // ID's
        // Encoder IDs have been set
        public int FRONT_LEFT_TRANS_ID = 1;
        public int FRONT_LEFT_ROT_ID = 8;
        public int  FRONT_LEFT_ROT_ENC_ID = 22;

        public int FRONT_RIGHT_TRANS_ID = 7;
        public int FRONT_RIGHT_ROT_ID = 3;
        public int FRONT_RIGHT_ROT_ENC_ID = 23;

        public int BACK_LEFT_TRANS_ID = 5;
        public int BACK_LEFT_ROT_ID = 4;
        public int BACK_LEFT_ROT_ENC_ID = 21;
        
        public int BACK_RIGHT_TRANS_ID = 9;
        public int BACK_RIGHT_ROT_ID = 6;
        public int BACK_RIGHT_ROT_ENC_ID = 20;


        public int PIGEON_ID = 0;


        // Inverse Booleans
        public boolean FRONT_LEFT_ROT_INVERSE = false;
        public boolean FRONT_LEFT_TRANS_INVERSE = false;

        public boolean FRONT_RIGHT_ROT_INVERSE = false;
        public boolean FRONT_RIGHT_TRANS_INVERSE = false;

        public boolean BACK_LEFT_ROT_INVERSE = false;
        public boolean BACK_LEFT_TRANS_INVERSE = false;

        public boolean BACK_RIGHT_ROT_INVERSE = false;
        public boolean BACK_RIGHT_TRANS_INVERSE = false;

        // Swerve Module Locations
        public Translation2d FRONT_LEFT_COORDS_METERS = null;
        public Translation2d FRONT_RIGHT_COORDS_METERS = null;
        public Translation2d BACK_LEFT_COORDS_METERS = null;
        public Translation2d BACK_RIGHT_COORDS_METERS = null;

        // PID Controllers
        public final PIDParameters FRONT_LEFT_ROT_PID =  new PIDParameters(0.35, 0, 0);;
        public final PIDParameters FRONT_RIGHT_ROT_PID =  new PIDParameters(0.35, 0, 0);;
        public final PIDParameters BACK_LEFT_ROT_PID =  new PIDParameters(0.35, 0, 0);;
        public final PIDParameters BACK_RIGHT_ROT_PID =  new PIDParameters(0.35, 0, 0);;

        public PIDParameters FRONT_LEFT_TRANS_PID = null;
        public PIDParameters FRONT_RIGHT_TRANS_PID = null;
        public PIDParameters BACK_LEFT_TRANS_PID = null;
        public PIDParameters BACK_RIGHT_TRANS_PID = null;

        public PIDParameters FRONT_LEFT_TRANS_CARPET_PID = null;
        public PIDParameters FRONT_RIGHT_TRANS_CARPET_PID = null;
        public PIDParameters BACK_LEFT_TRANS_CARPET_PID = null;
        public PIDParameters BACK_RIGHT_TRANS_CARPET_PID = null;


        public SwerveDriveKinematics SWERVE_KINEMATICS = null;


        // Swerve Modules and Other Hardware
        public  SwerveModuleAbstract FRONT_LEFT_MODULE = null;

        public  SwerveModuleAbstract FRONT_RIGHT_MODULE = null;

        public  SwerveModuleAbstract BACK_LEFT_MODULE = null;

        public  SwerveModuleAbstract BACK_RIGHT_MODULE = null;


        public  SwerveModuleAbstract[] SWERVE_MODULES = {FRONT_LEFT_MODULE, 
                FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE};

        public  PIDParameters[] SWERVE_MODULE_PIDs = {FRONT_LEFT_ROT_PID,
                FRONT_RIGHT_ROT_PID, BACK_LEFT_ROT_PID, BACK_RIGHT_ROT_PID};

}
