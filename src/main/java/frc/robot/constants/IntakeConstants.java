// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Handy Constants when dealing with the intake */
public class IntakeConstants {
    private IntakeConstants() {
        throw new IllegalStateException("Constants Class");
    }
    public static final int INTAKE_SPARK_ID_INNER = 30; //Done
    public static final boolean INTAKE_INVERTED_INNER = false;
    public static final int INTAKE_SPARK_ID_OUTER = 36; //Done
    public static final boolean INTAKE_INVERTED_OUTER = true; // will change later https://github.com/FRC-1294/frc2024/issues/239
    public static final int SMART_CURRENT_LIMIT_INNER = 0; // will change later https://github.com/FRC-1294/frc2024/issues/239
    public static final int SMART_CURRENT_LIMIT_OUTER = 0; // will change later https://github.com/FRC-1294/frc2024/issues/239
    public static final int INTAKE_BEAMBREAK_ID = 1;
    public static final double INNER_INTAKE_SPEED_AQUIRE = 0.6;
    public static final double INNER_INTAKE_SPEED_HANDOFF = 0.6;
    public static final double OUTER_INTAKE_SPEED_ACTIVE = 0.8;
    public static final double PASSIVE_INTAKE_SPEED = 0;
    public static final double HANDOFF_TIMER_MULTIPLIER = 0.1;
}
