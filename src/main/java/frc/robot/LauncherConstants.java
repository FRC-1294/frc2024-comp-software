// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class LauncherConstants {
  public static final double FLYWHEEL_TOLERANCE = 0.01; //TBD all random numbers for now

  public static final double[] LAUNCHER_MAIN_PID = new double [] {0, 0, 0, 0, 0};
  //public static final double[] LAUNCHER_ROLLER_PID = new double [] {0.0, 0.0, 0.0, 0.0};

  public static final int MAIN_FLYWHEEL_ID = 1;
  public static final int ROLLER_FLYWHEEL_ID = 2;

  public static final int INDEXER_ID = 3;

  public static final String mainFlyWheelCANbus = "x";
  public static final String rollerFlyWheelCANbus = "y";

  public static final int ABS_LAUNCHER_ENCODER_ID = 4;

  public static final int ENCODER_FLYWHEEL_INCREMENT = 2000; //ask probably
}
