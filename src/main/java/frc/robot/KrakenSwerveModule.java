// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants.KrakenSwerveConstants;
import frc.robot.constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class KrakenSwerveModule {
     // Parameters
    private final int mRotID;
    private final int mTransID;
    private final int mRotEncoderID;
    private final boolean mRotInverse;
    private final boolean mTransInverse;
    private final PIDController mRotPID;

    // Motors
    private final CANSparkMax mRotMotor;
    private final TalonFX mTransMotor;
    
    // Encoders
    private final CANcoder mRotEncoder;
    private final RelativeEncoder mRotRelativeEncoder;

    // Public Debug Values
    private double mPIDOutput = 0.0;
    private double mDesiredRadians = 0.0;

    /**
     * Creates a new Swerve Module with a Neo for Rotation and a Kraken for Translation
     * @param rotID Id of the Rotation Motor
     * @param transID Id of the Translation Motor
     * @param rotEncoderID Id of the Absolute Encoder (CANCoder)
     * @param rotInverse Is the rotation motor inversed
     * @param transInverse Is the translation motor inversed
     * @param rotPID the PIDController for rotation
     */
    public KrakenSwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse, boolean transInverse, PIDController rotPID) {
        mRotID = rotID;
        mTransID = transID;
        mRotEncoderID = rotEncoderID;
        mRotInverse = rotInverse;
        mTransInverse = transInverse;
        mRotPID = rotPID;
        mRotPID.enableContinuousInput(-Math.PI, Math.PI);

        mRotMotor = new CANSparkMax(mRotID, MotorType.kBrushless);
        mTransMotor = new TalonFX(mTransID);
        
        mRotEncoder = new CANcoder(rotEncoderID, "SWERVE_ENC");
        mRotRelativeEncoder = mRotMotor.getEncoder();
        mRotRelativeEncoder.setPosition(0);
        mTransMotor.setPosition(0);

        mRotMotor.setInverted(rotInverse);
        mTransMotor.setInverted(transInverse);

        mRotMotor.setIdleMode(IdleMode.kBrake);
        mTransMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * @return The motor controller's applied output duty cycle.
     */
    public double getAppliedOutput() {
        return mRotMotor.getAppliedOutput();
    }

    /**
     * Set the Rotation Motor Speed with no PID
     * @param speed from -1.0 to 1.0
     */
    public void setTransMotorRaw(double speed) {
        mTransMotor.set(speed);
    }

    /**
     * Set the Rotation Motor Speed with no PID
     * @param speed from -1.0 to 1.0
     */
    public void setRotMotorRaw(double speed) {
        mRotMotor.set(speed);
    }

    /**
     * @return Returns rotation position in radians
     */
    private double getRotPositionRaw() {
        return mRotEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    }

    /**
     * @return Returns number rotations of translation motor BEFORE GEAR RATIO
     */
    private double getTransPositionRaw() {
        return mTransMotor.getPosition().getValueAsDouble();
    }

    public double getRotPosition() {
        return getRotPositionRaw() * KrakenSwerveConstants.ABS_ENC_GEAR_RATIO_ROT;
    }

    public double getTransPosition() {
        return getTransPositionRaw() * KrakenSwerveConstants.TRANS_GEAR_RATIO_ROT * KrakenSwerveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    /**
     * @return Velocity in Rotations per minute
     */
    private double getTransVelocityRaw() {
        return mTransMotor.getVelocity().getValueAsDouble()/60;
    }

    public double getRotRelativePosition() {
        return mRotRelativeEncoder.getPosition() * KrakenSwerveConstants.REL_ENC_GEAR_RATIO_ROT;
    }

    public double getTransVelocity() {
        return getTransVelocityRaw() * SwerveConstants.TRANS_RPM_TO_MPS;
    }

    public SwerveModulePosition getModulePos() {
        return new SwerveModulePosition(getTransPosition(), Rotation2d.fromRadians(getRotPosition()));
    }

    
}
