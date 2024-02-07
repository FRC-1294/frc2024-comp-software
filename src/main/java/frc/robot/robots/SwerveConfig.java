// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.swerve.SwerveModuleAbstract;
import frc.robot.Util.PIDConstants;

/** Add your docs here. */
public class SwerveConfig {
        public SwerveConfig() {

        }
        public Pigeon2 PIGEON = null;
        public double TRANS_GEAR_RATIO_ROT = -1;
        public double REL_ENC_GEAR_RATIO_ROT = -1;
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
        public  int FRONT_LEFT_TRANS_ID = 0;
        public  int FRONT_LEFT_ROT_ID = 0;
        public  int FRONT_LEFT_ROT_ENC_ID = 0;

        public  int FRONT_RIGHT_TRANS_ID = 0;
        public  int FRONT_RIGHT_ROT_ID = 0;
        public  int FRONT_RIGHT_ROT_ENC_ID = 0;

        public  int BACK_LEFT_TRANS_ID = 0;
        public  int BACK_LEFT_ROT_ID = 0;
        public  int BACK_LEFT_ROT_ENC_ID = 0;

        public  int BACK_RIGHT_TRANS_ID = 0;
        public  int BACK_RIGHT_ROT_ID = 0;
        public  int BACK_RIGHT_ROT_ENC_ID = 0;


        public  int PIGEON_ID = 0;


        // Inverse Booleans
        public  boolean FRONT_LEFT_ROT_INVERSE = false;
        public  boolean FRONT_LEFT_TRANS_INVERSE = false;

        public  boolean FRONT_RIGHT_ROT_INVERSE = false;
        public  boolean FRONT_RIGHT_TRANS_INVERSE = false;

        public  boolean BACK_LEFT_ROT_INVERSE = false;
        public  boolean BACK_LEFT_TRANS_INVERSE = false;

        public  boolean BACK_RIGHT_ROT_INVERSE = false;
        public  boolean BACK_RIGHT_TRANS_INVERSE = false;

        // Swerve Module Locations
        public  Translation2d FRONT_LEFT_COORDS_METERS = null;
        public  Translation2d FRONT_RIGHT_COORDS_METERS = null;
        public  Translation2d BACK_LEFT_COORDS_METERS = null;
        public  Translation2d BACK_RIGHT_COORDS_METERS = null;

        // PID Controllers
        public  PIDConstants FRONT_LEFT_ROT_PID = null;
        public  PIDConstants FRONT_RIGHT_ROT_PID = null;
        public  PIDConstants BACK_LEFT_ROT_PID = null;
        public  PIDConstants BACK_RIGHT_ROT_PID = null;

        public  PIDConstants FRONT_LEFT_TRANS_PID = null;
        public  PIDConstants FRONT_RIGHT_TRANS_PID = null;
        public  PIDConstants BACK_LEFT_TRANS_PID = null;
        public  PIDConstants BACK_RIGHT_TRANS_PID = null;

        public  PIDConstants FRONT_LEFT_TRANS_CARPET_PID = null;
        public  PIDConstants FRONT_RIGHT_TRANS_CARPET_PID = null;
        public  PIDConstants BACK_LEFT_TRANS_CARPET_PID = null;
        public  PIDConstants BACK_RIGHT_TRANS_CARPET_PID = null;


        public  SwerveDriveKinematics SWERVE_KINEMATICS = null;


        // Swerve Modules and Other Hardware
        public  SwerveModuleAbstract FRONT_LEFT_MODULE = null;

        public  SwerveModuleAbstract FRONT_RIGHT_MODULE = null;

        public  SwerveModuleAbstract BACK_LEFT_MODULE = null;

        public  SwerveModuleAbstract BACK_RIGHT_MODULE = null;


        public  SwerveModuleAbstract[] SWERVE_MODULES = {FRONT_LEFT_MODULE, 
                FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE};

        public  PIDConstants[] SWERVE_MODULE_PIDs = {FRONT_LEFT_ROT_PID,
                FRONT_RIGHT_ROT_PID, BACK_LEFT_ROT_PID, BACK_RIGHT_ROT_PID};

}
