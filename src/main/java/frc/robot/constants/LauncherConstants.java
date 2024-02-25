// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class LauncherConstants {

  //IDs
  public static final int LEADER_FLYWHEEL_ID = 40; //TODO:https://github.com/FRC-1294/frc2024/issues/241
  public static final int FOLLOWER_FLYWHEEL_ID = 41; //TODO:https://github.com/FRC-1294/frc2024/issues/241

  public static final int INDEXER_ID = 31; //TODO:https://github.com/FRC-1294/frc2024/issues/241
  public static final int BEAMBREAK_ID = 3; //TODO:https://github.com/FRC-1294/frc2024/issues/241

  //current flywheel mode
  public enum LauncherMode {
    SPEAKER, AMP, TRAP, OFF, PASSIVE;
  }

  //flywheel setpoints
  public enum LauncherState {

    SPEAKER_DEFAULT(LauncherConstants.FLYWHEEL_MAX_VELOCITY), //TODO:https://github.com/FRC-1294/frc2024/issues/241

    AMP_DEFAULT(5000), //TODO:https://github.com/FRC-1294/frc2024/issues/241

    TRAP_DEFAULT(1000), //TODO:https://github.com/FRC-1294/frc2024/issues/241

    PASSIVE_DEFAULT(5000);

    public final double velocity;

    private LauncherState(double velocity) {
      this.velocity = velocity;
    }
  }

  //indexer setpoint
  public static final double INDEXER_VELOCITY_DEFAULT = 0.3; //TODO:https://github.com/FRC-1294/frc2024/issues/241
  
  public static final PIDController LAUNCHER_PID_CONTROLLER = new PIDController(0.1, 0, 0); //TODO:https://github.com/FRC-1294/frc2024/issues/241

  public static final SimpleMotorFeedforward LAUNCHER_FF_CONTROLLER = new SimpleMotorFeedforward(0, 12/LauncherConstants.FLYWHEEL_MAX_VELOCITY, 0); //TODO:https://github.com/FRC-1294/frc2024/issues/241

  public static final double FLYWHEEL_TOLERANCE = 10; //TODO:https://github.com/FRC-1294/frc2024/issues/241

  public static final double FLYWHEEL_MAX_VELOCITY = 12000; //rpm
  public static final double FLYWHEEL_SENSOR_TO_MECHANISM = 2;

  public static final boolean INDEXER_IS_INVERTED = false;

}
