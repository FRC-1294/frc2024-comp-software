// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.swerve.RevSwerveModule;
import frc.robot.Util.PIDConstants;

/** Add your docs here. */
public class PracticeBotSwerveConfig extends SwerveConfig{
        public PracticeBotSwerveConfig() {
                super();
        }

        public void config() {
            TRANS_GEAR_RATIO_ROT = 1 / 6.75;
            REL_ENC_GEAR_RATIO_ROT = 1 / 12.8;
            ABS_ENC_GEAR_RATIO_ROT = 1;

            // Conversion Factors
            WHEEL_DIAMETER_METERS = .1016;
            WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

            // Physical Max
            PHYSICAL_MAX_SPEED_MPS = 4.625;

            TELE_MAX_SPEED_MPS = 4.625;
            TELE_MAX_ROT_SPEED_RAD_SEC = 2 * Math.PI;

            TRACK_WIDTH_METERS = .495;
            TRACK_LENGTH_METERS = .495;
            // ID's
            // Encoder IDs have been set
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


            PIGEON_ID = 25;



         // Inverse s
            FRONT_LEFT_ROT_INVERSE = false;
            FRONT_LEFT_TRANS_INVERSE = true;

            FRONT_RIGHT_ROT_INVERSE = false;
            FRONT_RIGHT_TRANS_INVERSE = true;

            BACK_LEFT_ROT_INVERSE = false;
            BACK_LEFT_TRANS_INVERSE = false;

            BACK_RIGHT_ROT_INVERSE = false;
            BACK_RIGHT_TRANS_INVERSE = true;


            // PID Controllers
            FRONT_LEFT_ROT_PID = new PIDConstants(0.35, 0, 0);
            FRONT_RIGHT_ROT_PID = new PIDConstants(0.35, 0, 0);
            BACK_LEFT_ROT_PID = new PIDConstants(0.35, 0, 0);
            BACK_RIGHT_ROT_PID = new PIDConstants(0.35, 0, 0);

            FRONT_LEFT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.0099795525893569,0.21090956596760593);
            FRONT_RIGHT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.008545182645320892,0.2099873992689063);
            BACK_LEFT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.0099795525893569,0.2099731767248498);
            BACK_RIGHT_TRANS_PID = new PIDConstants(0.1, 0, 0,0.01037629321217537,0.21397894176465337);

            FRONT_LEFT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010559404268860817,0.2143581312619372);
            FRONT_RIGHT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010345774702727795,0.2147905172579474);
            BACK_LEFT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.011993774212896824,0.21911519031435608);
            BACK_RIGHT_TRANS_CARPET_PID = new PIDConstants(0.1, 0, 0,0.010132145136594772,0.21575530586570696);



            // Swerve Modules and Other Hardware
            FRONT_LEFT_MODULE = new RevSwerveModule(FRONT_LEFT_ROT_ID,
                           FRONT_LEFT_TRANS_ID, FRONT_LEFT_ROT_ENC_ID, FRONT_LEFT_ROT_INVERSE,
                           FRONT_LEFT_TRANS_INVERSE, FRONT_LEFT_ROT_PID,FRONT_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                           WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

            FRONT_RIGHT_MODULE = new RevSwerveModule(FRONT_RIGHT_ROT_ID,
                           FRONT_RIGHT_TRANS_ID, FRONT_RIGHT_ROT_ENC_ID, FRONT_RIGHT_ROT_INVERSE,
                           FRONT_RIGHT_TRANS_INVERSE, FRONT_RIGHT_ROT_PID,FRONT_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                           WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

            BACK_LEFT_MODULE = new RevSwerveModule(BACK_LEFT_ROT_ID,
                           BACK_LEFT_TRANS_ID, BACK_LEFT_ROT_ENC_ID, BACK_LEFT_ROT_INVERSE,
                           BACK_LEFT_TRANS_INVERSE, BACK_LEFT_ROT_PID,BACK_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                           WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

            BACK_RIGHT_MODULE = new RevSwerveModule(BACK_RIGHT_ROT_ID,
                           BACK_RIGHT_TRANS_ID, BACK_RIGHT_ROT_ENC_ID, BACK_RIGHT_ROT_INVERSE,
                           BACK_RIGHT_TRANS_INVERSE, BACK_RIGHT_ROT_PID,BACK_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                           WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);
        }

       

}
