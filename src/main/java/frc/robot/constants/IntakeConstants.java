// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Handy Constants when dealing with the intake */
public class IntakeConstants {
    private IntakeConstants() {
        throw new IllegalStateException("Constants Class");
    }
    public static final int INTAKE_SPARK_ID = 21;
    public static final boolean INTAKE_INVERTED = true;
    public static final int SMART_CURRENT_LIMIT = 0;
    public static final int INTAKE_BEAMBREAK_ID = 0;
    public static final double INTAKE_SPEED = 1.0;
}
