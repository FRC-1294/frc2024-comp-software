package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

    // Hardware
    // Motor Controllers
    protected final CANSparkMax mRotMotor;
    // Encoders
    private final CANcoder mRotEncoder;
    private final RelativeEncoder mRotRelativeEncoder;
    protected final double mPhysicalMaxSpeedMPS;
    private final double mAbsEncoderGearRatio;
    private final double mRelEncoderGearRatio;
    

    // Public Debugging Values
    private double mPIDOutput = 0.0;
    private double mDesiredRadians = 0.0;

    protected SwerveModuleAbstract(int rotID, int transID, int rotEncoderID, boolean rotInverse,
        boolean transInverse, PIDConstants rotPID, PIDConstants transPID, double transGearRatio,
        double wheelCircumference, double physicalMaxSpeed, double absEncGearRatio, double relEncoderGearRatio) {
            mRotID = rotID;
            mTransID = transID;
            mRotEncoderID = rotEncoderID;
            mRotInverse = rotInverse;
            mTransInverse = transInverse;

            mPhysicalMaxSpeedMPS = physicalMaxSpeed;
            mAbsEncoderGearRatio = absEncGearRatio;
            mRelEncoderGearRatio = relEncoderGearRatio;

            mRotPID = rotPID.toWPIController();
            mTransPID = transPID.toWPIController();
            mTransFF = transPID.toWPIMotorFeedForward();


            // ----Setting Hardware
            // Motor Controllers
            mRotMotor = new CANSparkMax(mRotID, CANSparkMax.MotorType.kBrushless);
            // Encoders
            mRotRelativeEncoder = mRotMotor.getEncoder();
            mRotEncoder = new CANcoder(mRotEncoderID,"SWERVE_ENC");
            mRotRelativeEncoder.setPosition(0);

            // ----Setting PID Parameters
            mRotPID.enableContinuousInput(-Math.PI, Math.PI);

            // ----Setting Inversion
            mRotMotor.setInverted(mRotInverse);

            mRotMotor.setIdleMode(IdleMode.kBrake);

            mRotMotor.burnFlash();
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

    public double getRotAppliedOutput() {
        return mRotMotor.getAppliedOutput();
    };

    public abstract void setTransMotorDutyCycleRaw(double speed);

    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    protected abstract void setTranslationVoltageRaw(double volts);

    public void setRotMotorRaw(double speed) {
        mRotMotor.set(speed);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        setDesiredState(desiredState);
    }

    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        mDesiredRadians = desiredState.angle.getRadians();
        mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());
        mRotMotor.set(mPIDOutput);
    }
    

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
     * @return Returns rotation position in radians
     */
    private double getRotPositionRaw() {
        return mRotEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    }

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public double getRotPosition() {
        return getRotPositionRaw() * mAbsEncoderGearRatio;
    }
    
    public double getRotRelativePosition() {
        return mRotRelativeEncoder.getPosition() * mRelEncoderGearRatio;
    }

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

    /**
     * Sets the mode of the rotation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode) {
        mRotMotor.setIdleMode(mode);
    }

    /**
     * Retrives the rotation PID output provided to the motors after desaturation and optimization
     */
    public double getPIDOutputRot() {
        return mPIDOutput;
    }

    /**
     * Retrives the desired radian setpoint of rotation of the motors after desaturation and
     * optimization.
     */
    public double getDesiredRadiansRot() {
        return mDesiredRadians;
    }

}
