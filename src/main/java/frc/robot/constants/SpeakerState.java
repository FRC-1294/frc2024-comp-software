// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public enum SpeakerState {

    SUBWOOFER(0,0,0),
    LINE(0,0,0),
    MIDNOTE(0,0,0),
    WING(0,0,0);
    public final double mWristAngleDeg;
    public final double mRadialDistanceMeters;
    public final double mShotToleranceMeters;

    SpeakerState(double wristAngleDeg, double radialDistanceMeters, double shotToleranceMeters) {
        mWristAngleDeg = wristAngleDeg;
        mRadialDistanceMeters = radialDistanceMeters;
        mShotToleranceMeters = shotToleranceMeters;
    }
}
