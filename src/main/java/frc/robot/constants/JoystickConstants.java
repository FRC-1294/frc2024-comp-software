// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class JoystickConstants {
    private JoystickConstants() {
        throw new IllegalStateException("Constants Class");
    }

    public static final int ROT_JOYSTICK_PORT = 0;
    public static final int TRANS_JOY_PORT = 2;
    public static final int XBOX_PORT = 1;

    public static final double DRIVE_PRECISION_X_DESATURATION = 2;
    public static final double DRIVE_PRECISION_Y_DESATURATION = 2;
    public static final double DRIVE_PRECISION_ROT_DESATURATION = 2;

    public static final double DRIVE_PRECISION_X_DEADZONE = 0.04;
    public static final double DRIVE_PRECISION_Y_DEADZONE = 0.04;
    public static final double DRIVE_PRECISION_ROT_DEADZONE = 0.02;

    public static final double DRIVE_REG_X_DEADZONE = 0.10;
    public static final double DRIVE_REG_Y_DEADZONE = 0.10;
    public static final double DRIVE_REG_ROT_DEADZONE = 0.07*1.5;

    public static final double XBOX_Y_DEADZONE = 0.1;

    public static final double XBOX_RUMBLE_SOFT = 0.2;
    public static final double XBOX_RUMBLE_MEDIUM = 0.6;
    public static final double XBOX_RUMBLE_VIGEROUS = 1;

}
