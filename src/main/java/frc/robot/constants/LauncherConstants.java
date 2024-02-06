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
  public static final int MAIN_FLYWHEEL_ID = 1; //TBD https://github.com/FRC-1294/frc2024/issues/241
  public static final int ROLLER_FLYWHEEL_ID = 2; //TBD https://github.com/FRC-1294/frc2024/issues/241

  public static final int INDEXER_ID = 3; //TBD https://github.com/FRC-1294/frc2024/issues/241

  public static final int BEAMBREAK_ID = 4; //TBD https://github.com/FRC-1294/frc2024/issues/241

  //current flywheel mode
  public enum LauncherMode {
    SPEAKER, AMP, TRAP, OFF;
  }

  //flywheel setpoints
  public enum LauncherState {

    SPEAKER_DEFAULT(300, 300), //TBD https://github.com/FRC-1294/frc2024/issues/241

    AMP_DEFAULT(50, 50), //TBD https://github.com/FRC-1294/frc2024/issues/241

    TRAP_DEFAULT(50, 50); //TBD https://github.com/FRC-1294/frc2024/issues/241

    public final double mainVelocity;
    public final double rollerVelocity;

    private LauncherState(double mainVelocity, double rollerVelocity) {
      this.mainVelocity = mainVelocity;
      this.rollerVelocity = rollerVelocity;
    }
  }

  //indexer setpoint
  public static final double INDEXER_VELOCITY_DEFAULT = 1; //TBD https://github.com/FRC-1294/frc2024/issues/241
  
  public static final PIDController LAUNCHER_PID_CONTROLLER = new PIDController(0, 0, 0, 0);

  public static final SimpleMotorFeedforward LAUNCHER_FF_CONTROLLER = new SimpleMotorFeedforward(0, 0, 0);

  public static final double FLYWHEEL_TOLERANCE = 0.01; //TBD https://github.com/FRC-1294/frc2024/issues/241

  public static final double FLYWHEEL_MAX_VELOCITY = 511.998046875; //rps

}