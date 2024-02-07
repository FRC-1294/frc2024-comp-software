package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Util.PIDConstants;

public class RevSwerveModule extends SwerveModuleAbstract{

    // Hardware
    // Motor Controllers
    private final CANSparkMax mRotMotor;
    private final CANSparkMax mTransMotor;
    // Encoders
    private final CANcoder mRotEncoder;
    private final RelativeEncoder mTransEncoder;
    private final RelativeEncoder mRotRelativeEncoder;
    private final double mPhysicalMaxSpeedMPS;
    private final double mAbsEncoderGearRatio;
    private final double mRelEncoderGearRatio;
    

    // Public Debugging Values
    private double mPIDOutput = 0.0;
    private double mDesiredRadians = 0.0;
    private double mDesiredVel = 0.0;
    private double mMaxAccel = 0.0;
    private double mCurAccel = 0.0;
    private double prevVel = 0.0;
    private double prevTS;
    public double feedforward=0;

    public RevSwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse,
    boolean transInverse, PIDConstants rotPID, PIDConstants transPID, double transGearRatio,
    double wheelCircumference, double physicalMaxSpeed, double absEncGearRatio, double relEncoderGearRatio) {
        // Setting Parameters
    
        super(rotID, transID, rotEncoderID, rotInverse, transInverse, rotPID, transPID);
        mPhysicalMaxSpeedMPS = physicalMaxSpeed;
        mAbsEncoderGearRatio = absEncGearRatio;
        mRelEncoderGearRatio = relEncoderGearRatio;

        // ----Setting Hardware
        // Motor Controllers
        mRotMotor = new CANSparkMax(mRotID, MotorType.kBrushless);
        mTransMotor = new CANSparkMax(mTransID, MotorType.kBrushless);
        mRotMotor.restoreFactoryDefaults();
        mTransMotor.restoreFactoryDefaults();

        // Encoders
        mRotEncoder = new CANcoder(mRotEncoderID,"SWERVE_ENC");
        mTransEncoder = mTransMotor.getEncoder();
        mRotRelativeEncoder = mRotMotor.getEncoder();
        mRotRelativeEncoder.setPosition(0);

        // ----Setting PID Parameters
        mRotPID.enableContinuousInput(-Math.PI, Math.PI);

        // ----Setting Inversion
        mRotMotor.setInverted(mRotInverse);
        mTransMotor.setInverted(mTransInverse);

        mTransMotor.setIdleMode(IdleMode.kBrake);
        mRotMotor.setIdleMode(IdleMode.kBrake);

        mTransEncoder.setPosition(0);

        mTransMotor.enableVoltageCompensation(12);
        mTransEncoder.setPositionConversionFactor(transGearRatio*wheelCircumference);
        mTransEncoder.setVelocityConversionFactor(transGearRatio*wheelCircumference/60);
        mTransEncoder.setPosition(0);

        burnSparks();
    }

    // ------------------- State Settings

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
    }

    public void setTransMotorDutyCycleRaw(double speed) {
        mTransMotor.set(speed);
    }

    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    public void setTranslationVoltageRaw(double volts) {
        mTransMotor.set(volts / mTransMotor.getVoltageCompensationNominalVoltage());
    }

    public void setRotMotorRaw(double speed) {
        mRotMotor.set(speed);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        if (isOpenLoop){
            if (Math.abs(desiredState.speedMetersPerSecond) < 0.0000000001) {
                stop();
                return;
            }

            // No turning motors over 90 degrees
            desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

            mDesiredVel = desiredState.speedMetersPerSecond;
            mTransMotor.set(mDesiredVel/mPhysicalMaxSpeedMPS);

            mDesiredRadians = desiredState.angle.getRadians();
            mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());
            mRotMotor.set(mPIDOutput); 
        }else{
            setDesiredState(desiredState);
        }

    }
    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        double curVel = getTransVelocity();
        mCurAccel = (Math.abs(curVel)-Math.abs(prevVel))/(Timer.getFPGATimestamp()-prevTS);
        if (mCurAccel>mMaxAccel){
            mMaxAccel = mCurAccel;
        }
        prevTS = Timer.getFPGATimestamp();
        prevVel = curVel;
        // Stops returning to original rotation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.0001) {
            stop();
            return;
        }

        // No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // PID Controller for both translation and rotation
        mDesiredVel = desiredState.speedMetersPerSecond;
        feedforward = mTransFF.calculate(mDesiredVel);
        double pidOutput = mTransPID.calculate(getTransVelocity(), mDesiredVel) / mPhysicalMaxSpeedMPS;
        mTransMotor.set(pidOutput + feedforward);

        mDesiredRadians = desiredState.angle.getRadians();
        mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());
        mRotMotor.set(mPIDOutput);
    }

    public void setPID(double degrees) {
        mPIDOutput = mRotPID.calculate(getRotPosition(), Math.toRadians(degrees));
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

    // -------------------- Get Raw Values

    /**
     * 
     * @return Returns number rotations of translation motor BEFORE GEAR RATIO
     */
    private double getTransPositionRaw() {
        return mTransEncoder.getPosition();
    }


    /**
     * 
     * @return Returns rotation position in radians
     */
    private double getRotPositionRaw() {
        return mRotEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    }

    /**
     * 
     * @return Returns velocity of translation motor BEFORE GEAR RATIO
     */
    private double getTransVelocityRaw() {
        return mTransEncoder.getVelocity();
    }

    // -------------------- Applying Conversions/Rollover

    /**
     * 
     * @return Returns translation motor AFTER GEAR RATIO and Meters
     */
    public double getTransPosition() {
        return getTransPositionRaw();
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
    public double getTransVelocity() {
        return getTransVelocityRaw();
    }

    /**
     * 
     * @return the PID setpoint of the translation's velocity in meters/sec
     */
    public double getTransVelocitySetpoint(){
        return mDesiredVel;
    }

    /**
     * @return the max acceleration in m/s^2 from the translation motor's initialization
     */
    public double getMaxAccel(){
        return mMaxAccel;
    }

    /**
     * @return the current acceleration in m/s^2 from the translation motor
     */
    public double getCurAccel(){
        return mCurAccel;
    }
    /**
     * 
     * @return Returns the applied voltage to the translation motor after nominal voltage
     *         compensation
     */
    public double getTransAppliedVolts() {
        return mTransMotor.getAppliedOutput() * mTransMotor.getVoltageCompensationNominalVoltage();
    }

    /**
     * Reset ONLY the translation encoder
     */
    public void resetEncoders() {
        mTransEncoder.setPosition(0);
    }

    /**
     * Stops the both motors
     */
    public void stop() {
        mTransMotor.set(0);
        mRotMotor.set(0);
    }

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getRotPIDController() {
        return mRotPID;
    }

    /**
     * Sets the mode of the translation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode) {
        mTransMotor.setIdleMode(mode);
    }

    /**
     * Permanently burnCs settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        mRotMotor.burnFlash();
        mTransMotor.burnFlash();
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

    /**
     * @return the nominal voltage amount after voltage compensation for the translation motor
     */
    public double getTranslationNominalVoltage(){
        return mTransMotor.getVoltageCompensationNominalVoltage();
    }
}
