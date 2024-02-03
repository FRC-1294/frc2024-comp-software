package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.Util.TalonControlType;

public class KrakenSwerveModule extends SwerveModuleAbstract{

    // Hardware
    // Motor Controllers
    private final CANSparkMax mRotMotor;
    private final TalonFX mTransMotor;
    // Encoders
    private final CANcoder mRotEncoder;
    private final RelativeEncoder mRotRelativeEncoder;
    private final TalonFXConfiguration mTransConfiguration;
    private final VelocityVoltage mVelocityVoltageSignal = new VelocityVoltage(0).withSlot(1);
    private final CoastOut mCoastOutSignal = new CoastOut();
    private final double mNominalVoltage = 12;
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

    public KrakenSwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse,
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
        mTransMotor = new TalonFX(transID, "DriveMotors");
        mTransConfiguration = new TalonFXConfiguration();

        mTransConfiguration.Feedback.SensorToMechanismRatio = transGearRatio *wheelCircumference;
        mTransConfiguration.withSlot0(transPID.toTalonConfiguration());

        mRotMotor.restoreFactoryDefaults();

        // Encoders
        mRotEncoder = new CANcoder(mRotEncoderID,"DriveMotors");
        mRotRelativeEncoder = mRotMotor.getEncoder();
        mRotRelativeEncoder.setPosition(0);

        // ----Setting PID Parameters
        mRotPID.enableContinuousInput(-Math.PI, Math.PI);

        // ----Setting Inversion
        mRotMotor.setInverted(mRotInverse);
        mTransMotor.setInverted(mTransInverse);

        mTransMotor.setNeutralMode(NeutralModeValue.Brake);
        mRotMotor.setIdleMode(IdleMode.kBrake);

        mTransMotor.setPosition(0);

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
        mTransMotor.setVoltage(volts);;
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

            // mDesiredVel = desiredState.speedMetersPerSecond;
            // mTransMotor.set(mDesiredVel/mPhysicalMaxSpeedMPS);

            mDesiredRadians = desiredState.angle.getRadians();
            mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());
            mRotMotor.set(mPIDOutput); 
        }else{
            setDesiredState(desiredState);
        }
        // System.out.println("Set Desired Module State Nano: " + ((double)System.nanoTime()/1000000-ts5));

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
        mTransMotor.setVoltage((pidOutput + feedforward)*mNominalVoltage);

        //Onboard PID + FF solution
        // mTransMotor.setControl(mVelocityVoltageSignal.withVelocity(mDesiredVel));

        // mDesiredRadians = desiredState.angle.getRadians();
        // mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());
        // mRotMotor.set(mPIDOutput);
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
        return mTransMotor.getPosition().getValueAsDouble();
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
        return mTransMotor.getVelocity().getValueAsDouble();
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
        return mTransMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Reset ONLY the translation encoder
     */
    public void resetEncoders() {
        mTransMotor.setPosition(0);
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
    public void setModeTrans(TalonControlType mode) {
        switch (mode) {
            case COAST_OUT:
                mTransMotor.setControl(mCoastOutSignal);
                break;
            case VELOCITY_VOLTAGE:
                mTransMotor.setControl(mVelocityVoltageSignal);
            default:
                break;
        }
    }

    /**
     * Permanently burnCs settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        mRotMotor.burnFlash();
        // mTransMotor.burnFlash();
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

    // /**
    //  * @return the nominal voltage amount after voltage compensation for the translation motor
    //  */
    public double getTransNominalVoltage(){
        return mNominalVoltage; 
    }
}
