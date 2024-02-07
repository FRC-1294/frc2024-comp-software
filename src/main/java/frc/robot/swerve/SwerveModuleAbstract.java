package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Util.PIDConstants;

/*
    This is a wrapper class that serves both as an abstract class and a parent class to the Rev and Kraken Swerve Module classes
 */
 public abstract class SwerveModuleAbstract{
    // Parameters
    protected final int mRotID;
    protected final int mTransID;
    protected final int mRotEncoderID;
    protected final boolean mRotInverse;
    protected final boolean mTransInverse;
    protected final PIDController mRotPID;
    protected final PIDController mTransPID;
    protected final SimpleMotorFeedforward mTransFF;

    protected SwerveModuleAbstract(int rotID, int transID, int rotEncoderID, boolean rotInverse,
            boolean transInverse, PIDConstants rotPID, PIDConstants transPID) {
    
            mRotID = rotID;
            mTransID = transID;
            mRotEncoderID = rotEncoderID;
            mRotInverse = rotInverse;
            mTransInverse = transInverse;

            mRotPID = rotPID.toWPIController();
            mTransPID = transPID.toWPIController();
            mTransFF = transPID.toWPIMotorFeedForward();
        }


    /**
     * 
     * @return a swerve module state object describing the current speed of the translation and
     *         rotation motors
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTransVelocity(), Rotation2d.fromRadians(getRotPosition()));
    }

    public abstract double getRotAppliedOutput();

    public abstract void setTransMotorDutyCycleRaw(double speed);

    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    protected abstract void setTranslationVoltageRaw(double volts);

    public abstract void setRotMotorRaw(double speed);

    public abstract void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public abstract void setDesiredState(SwerveModuleState desiredState);

    public abstract void setPID(double degrees);

    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the
     *         form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getModulePos() {
        return new SwerveModulePosition(getTransPosition(),
                Rotation2d.fromRadians(getRotPosition()));
    }

    // -------------------- Applying Conversions/Rollover

    /**
     * 
     * @return Returns translation motor AFTER GEAR RATIO and Meters
     */
    
    public abstract double getTransPosition();

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public abstract double getRotPosition();

    public abstract double getRotRelativePosition();

    /**
     * 
     * @return Returns velocity of translation motor with conversion from the CANcoder
     */
    public abstract double getTransVelocity();
    /**
     * 
     * @return the PID setpoint of the translation's velocity in meters/sec
     */
    public abstract double getTransVelocitySetpoint();

    /**
     * 
     * @return Returns the applied voltage to the translation motor after nominal voltage
     *         compensation
     */
    public abstract double getTransAppliedVolts();

    /**
     * 
     * @return Returns the applied voltage to the translation motor after nominal voltage
     *         compensation
     */
    public abstract double getTransNominalVoltage();

    /**
     * Reset ONLY the translation encoder
     */
    public abstract void resetEncoders();

    /**
     * Stops the both motors
     */
    public abstract void stop();

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getRotPIDController() {
        return mRotPID;
    }

}
