// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import frc.robot.swerve.KrakenSwerveModule;
import frc.robot.Util.PIDParameters;

/** Add your docs here. */
public class CompetitionBotSwerveConfig extends SwerveConfig{
         public CompetitionBotSwerveConfig() {
                super();

        }

        public void config() {
                TRANS_GEAR_RATIO_ROT = 1 / 6.75;
                REL_ENC_GEAR_RATIO_ROT = 1 / 12.8;
                ABS_ENC_GEAR_RATIO_ROT = 1;
      
                // Conversion Factors
                WHEEL_DIAMETER_METERS = .102;
                WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
                TRANS_RPM_TO_MPS =
                (TRANS_GEAR_RATIO_ROT * WHEEL_CIRCUMFERENCE_METERS) / 60;
      
                // Physical Max
                PHYSICAL_MAX_SPEED_MPS = 4.625;
        
                TELE_MAX_SPEED_MPS = 4.625;
                TELE_MAX_ROT_SPEED_RAD_SEC = 2 * Math.PI;
        
                TELE_MAX_ACC_MPS2 = 5.0;
                TELE_MAX_ROT_ACC_RAD_SEC2 = 4 * Math.PI;

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

                // Inverse Booleans
                FRONT_LEFT_ROT_INVERSE = false;
                FRONT_LEFT_TRANS_INVERSE = true;

                FRONT_RIGHT_ROT_INVERSE = false;
                FRONT_RIGHT_TRANS_INVERSE = true;

                BACK_LEFT_ROT_INVERSE = false;
                BACK_LEFT_TRANS_INVERSE = true;

                BACK_RIGHT_ROT_INVERSE = false;
                BACK_RIGHT_TRANS_INVERSE = true;



                // PID Controllers
                FRONT_LEFT_ROT_PID = new PIDParameters(0.35, 0, 0);
                FRONT_RIGHT_ROT_PID = new PIDParameters(0.35, 0, 0);
                BACK_LEFT_ROT_PID = new PIDParameters(0.35, 0, 0);
                BACK_RIGHT_ROT_PID = new PIDParameters(0.35, 0, 0);

                FRONT_LEFT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.21090956596760593);
                FRONT_RIGHT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.2099873992689063);
                BACK_LEFT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.2099731767248498);
                BACK_RIGHT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.21397894176465337);

                FRONT_LEFT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.20881603812002797);
                FRONT_RIGHT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.21095308664591073);
                BACK_LEFT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.20998656795835002);
                BACK_RIGHT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.011666666666666667,0.20972130028356997);


                // Swerve Modules and Other Hardware
                FRONT_LEFT_MODULE = new KrakenSwerveModule(FRONT_LEFT_ROT_ID,
                                FRONT_LEFT_TRANS_ID, FRONT_LEFT_ROT_ENC_ID, FRONT_LEFT_ROT_INVERSE,
                                FRONT_LEFT_TRANS_INVERSE, FRONT_LEFT_ROT_PID,FRONT_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                FRONT_RIGHT_MODULE = new KrakenSwerveModule(FRONT_RIGHT_ROT_ID,
                                FRONT_RIGHT_TRANS_ID, FRONT_RIGHT_ROT_ENC_ID, FRONT_RIGHT_ROT_INVERSE,
                                FRONT_RIGHT_TRANS_INVERSE, FRONT_RIGHT_ROT_PID,FRONT_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                BACK_LEFT_MODULE = new KrakenSwerveModule(BACK_LEFT_ROT_ID,
                                BACK_LEFT_TRANS_ID, BACK_LEFT_ROT_ENC_ID, BACK_LEFT_ROT_INVERSE,
                                BACK_LEFT_TRANS_INVERSE, BACK_LEFT_ROT_PID,BACK_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                BACK_RIGHT_MODULE = new KrakenSwerveModule(BACK_RIGHT_ROT_ID,
                                BACK_RIGHT_TRANS_ID, BACK_RIGHT_ROT_ENC_ID, BACK_RIGHT_ROT_INVERSE,
                                BACK_RIGHT_TRANS_INVERSE, BACK_RIGHT_ROT_PID,BACK_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                
        }



}
