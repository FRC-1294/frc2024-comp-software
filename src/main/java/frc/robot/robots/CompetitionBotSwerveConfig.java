// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.swerve.KrakenSwerveModule;
import frc.robot.Util.PIDConstants;

/** Add your docs here. */
public class CompetitionBotSwerveConfig {
        public CompetitionBotSwerveConfig() {

        }

        public static final double TRANS_GEAR_RATIO_ROT = 1 / 6.75;
        public static final double REL_ENC_GEAR_RATIO_ROT = 1 / 12.8;
        public static final double ABS_ENC_GEAR_RATIO_ROT = 1;
      
        // Conversion Factors
        public static final double WHEEL_DIAMETER_METERS = .1016;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TRANS_RPM_TO_MPS =
            (TRANS_GEAR_RATIO_ROT * WHEEL_CIRCUMFERENCE_METERS) / 60;
      
        // Physical Max
        public static final double PHYSICAL_MAX_SPEED_MPS = 4.625;
      
        public static final double TELE_MAX_SPEED_MPS = 4.625;
        public static final double TELE_MAX_ROT_SPEED_RAD_SEC = 2 * Math.PI;
      
        public static final double TELE_MAX_ACC_MPS2 = 5.0;
        public static final double TELE_MAX_ROT_ACC_RAD_SEC2 = 4 * Math.PI;

        public static final double TRACK_WIDTH_METERS = .495;
        public static final double TRACK_LENGTH_METERS = .495;

        // ID's
        // Encoder IDs have been set
        private final int FRONT_LEFT_TRANS_ID = 1;
        private final int FRONT_LEFT_ROT_ID = 8;
        private final int FRONT_LEFT_ROT_ENC_ID = 22;

        private final int FRONT_RIGHT_TRANS_ID = 7;
        private final int FRONT_RIGHT_ROT_ID = 3;
        private final int FRONT_RIGHT_ROT_ENC_ID = 23;

        private final int BACK_LEFT_TRANS_ID = 5;
        private final int BACK_LEFT_ROT_ID = 4;
        private final int BACK_LEFT_ROT_ENC_ID = 21;

        private final int BACK_RIGHT_TRANS_ID = 9;
        private final int BACK_RIGHT_ROT_ID = 6;
        private final int BACK_RIGHT_ROT_ENC_ID = 20;


        public final int PIGEON_ID = 25;


        // Inverse Booleans
        private final boolean FRONT_LEFT_ROT_INVERSE = false;
        private final boolean FRONT_LEFT_TRANS_INVERSE = true;

        private final boolean FRONT_RIGHT_ROT_INVERSE = false;
        private final boolean FRONT_RIGHT_TRANS_INVERSE = true;

        private final boolean BACK_LEFT_ROT_INVERSE = false;
        private final boolean BACK_LEFT_TRANS_INVERSE = false;

        private final boolean BACK_RIGHT_ROT_INVERSE = false;
        private final boolean BACK_RIGHT_TRANS_INVERSE = true;

        // Swerve Module Locations
        public final Translation2d FRONT_LEFT_COORDS_METERS =
                        new Translation2d(TRACK_LENGTH_METERS / 2,
                                        TRACK_WIDTH_METERS / 2);
        public final Translation2d FRONT_RIGHT_COORDS_METERS =
                        new Translation2d(TRACK_LENGTH_METERS / 2,
                                        -TRACK_WIDTH_METERS / 2);
        public final Translation2d BACK_LEFT_COORDS_METERS =
                        new Translation2d(-TRACK_LENGTH_METERS / 2,
                                        TRACK_WIDTH_METERS / 2);
        public final Translation2d BACK_RIGHT_COORDS_METERS =
                        new Translation2d(-TRACK_LENGTH_METERS / 2,
                                        -TRACK_WIDTH_METERS / 2);

        // PID Controllers
        public final PIDConstants FRONT_LEFT_ROT_PID = new PIDConstants(0.35, 0, 0);
        public final PIDConstants FRONT_RIGHT_ROT_PID = new PIDConstants(0.35, 0, 0);
        public final PIDConstants BACK_LEFT_ROT_PID = new PIDConstants(0.35, 0, 0);
        public final PIDConstants BACK_RIGHT_ROT_PID = new PIDConstants(0.35, 0, 0);

        public final PIDConstants FRONT_LEFT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.0099795525893569,0.21090956596760593);
        public final PIDConstants FRONT_RIGHT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.008545182645320892,0.2099873992689063);
        public final PIDConstants BACK_LEFT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.0099795525893569,0.2099731767248498);
        public final PIDConstants BACK_RIGHT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.01037629321217537,0.21397894176465337);

        public final PIDConstants FRONT_LEFT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010559404268860817,0.2143581312619372);
        public final PIDConstants FRONT_RIGHT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010345774702727795,0.2147905172579474);
        public final PIDConstants BACK_LEFT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.011993774212896824,0.21911519031435608);
        public final PIDConstants BACK_RIGHT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010132145136594772,0.21575530586570696);


        public final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        FRONT_LEFT_COORDS_METERS, FRONT_RIGHT_COORDS_METERS,
                        BACK_LEFT_COORDS_METERS, BACK_RIGHT_COORDS_METERS);


        // Swerve Modules and Other Hardware
        public final KrakenSwerveModule FRONT_LEFT_MODULE = new KrakenSwerveModule(FRONT_LEFT_ROT_ID,
                        FRONT_LEFT_TRANS_ID, FRONT_LEFT_ROT_ENC_ID, FRONT_LEFT_ROT_INVERSE,
                        FRONT_LEFT_TRANS_INVERSE, FRONT_LEFT_ROT_PID,FRONT_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                        WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

        public final KrakenSwerveModule FRONT_RIGHT_MODULE = new KrakenSwerveModule(FRONT_RIGHT_ROT_ID,
                        FRONT_RIGHT_TRANS_ID, FRONT_RIGHT_ROT_ENC_ID, FRONT_RIGHT_ROT_INVERSE,
                        FRONT_RIGHT_TRANS_INVERSE, FRONT_RIGHT_ROT_PID,FRONT_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                        WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

        public final KrakenSwerveModule BACK_LEFT_MODULE = new KrakenSwerveModule(BACK_LEFT_ROT_ID,
                        BACK_LEFT_TRANS_ID, BACK_LEFT_ROT_ENC_ID, BACK_LEFT_ROT_INVERSE,
                        BACK_LEFT_TRANS_INVERSE, BACK_LEFT_ROT_PID,BACK_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                        WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

        public final KrakenSwerveModule BACK_RIGHT_MODULE = new KrakenSwerveModule(BACK_RIGHT_ROT_ID,
                        BACK_RIGHT_TRANS_ID, BACK_RIGHT_ROT_ENC_ID, BACK_RIGHT_ROT_INVERSE,
                        BACK_RIGHT_TRANS_INVERSE, BACK_RIGHT_ROT_PID,BACK_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                        WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);


        public final KrakenSwerveModule[] SWERVE_MODULES = {FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE,
                        BACK_LEFT_MODULE, BACK_RIGHT_MODULE};

        public final PIDConstants[] SWERVE_MODULE_PIDs = {FRONT_LEFT_ROT_PID,
                        FRONT_RIGHT_ROT_PID, BACK_LEFT_ROT_PID, BACK_RIGHT_ROT_PID};

}
