package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Util;
import frc.robot.Util.PIDConstants;
import frc.robot.constants.SwerveConstants;

 public class SwerveModuleAbstract{
    // Parameters
    protected final int mRotID;
    protected final int mTransID;
    protected final int mRotEncoderID;
    protected final boolean mRotInverse;
    protected final boolean mTransInverse;
    protected final PIDController mRotPID;
    protected final PIDController mTransPID;
    protected final SimpleMotorFeedforward mTransFF;

    public SwerveModuleAbstract(int rotID, int transID, int rotEncoderID, boolean rotInverse,
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

    public double getRotAppliedOutput(){return -9999999.9;};

    public void setTransMotorDutyCycleRaw(double speed){};

    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    protected void setTranslationVoltageRaw(double volts){};

    public void setRotMotorRaw(double speed){};

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){};
    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState){};

    public void setPID(double degrees){};


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
    
    public double getTransPosition(){return -999999.9999;}

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public double getRotPosition(){return -999999.9999;}

    public double getRotRelativePosition(){return -999999.9999;}

    /**
     * 
     * @return Returns velocity of translation motor with conversion from the CANcoder
     */
    public double getTransVelocity(){return -999999.9999;}

    /**
     * 
     * @return the PID setpoint of the translation's velocity in meters/sec
     */
    public double getTransVelocitySetpoint(){return -999999.9999;}

    /**
     * 
     * @return Returns the applied voltage to the translation motor after nominal voltage
     *         compensation
     */
    public double getTransAppliedVolts(){return -999999.9999;}

    /**
     * Reset ONLY the translation encoder
     */
    public void resetEncoders(){}

    /**
     * Stops the both motors
     */
    public void stop(){}

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getRotPIDController() {
        return mRotPID;
    }

}
