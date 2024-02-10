package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Util.PIDConstants;

public class RevSwerveModule extends SwerveModuleAbstract{

    // Hardware
    // Motor Controllers
    private final CANSparkMax mTransMotor;
    // Encoders
    private final RelativeEncoder mTransEncoder;

    

    // Public Debugging Values
    private double mDesiredVel = 0.0;

    public RevSwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse,
    boolean transInverse, PIDConstants rotPID, PIDConstants transPID, double transGearRatio,
    double wheelCircumference, double physicalMaxSpeed, double absEncGearRatio, double relEncoderGearRatio) {
        // Setting Parameters
    
        super(rotID, transID, rotEncoderID, rotInverse, transInverse, rotPID, transPID, transGearRatio, wheelCircumference, physicalMaxSpeed, absEncGearRatio, relEncoderGearRatio);


        // ----Setting Hardware
        // Motor Controllers
        mTransMotor = new CANSparkMax(mTransID, MotorType.kBrushless);
        mTransMotor.restoreFactoryDefaults();

        // Encoders
        mTransEncoder = mTransMotor.getEncoder();

        // ----Setting PID Parameters
        mRotPID.enableContinuousInput(-Math.PI, Math.PI);

        // ----Setting Inversion
        mTransMotor.setInverted(mTransInverse);

        mTransMotor.setIdleMode(IdleMode.kBrake);

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
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTransVelocity(), Rotation2d.fromRadians(getRotPosition()));
    }


    @Override
    public void setTransMotorDutyCycleRaw(double speed) {
        mTransMotor.set(speed);
    }

    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    @Override
    public void setTranslationVoltageRaw(double volts) {
        mTransMotor.set(volts / mTransMotor.getVoltageCompensationNominalVoltage());
    }



    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        super.setDesiredState(desiredState);
        if (isOpenLoop){
            if (Math.abs(desiredState.speedMetersPerSecond) < 0.0000000001) {
                stop();
                return;
            }

            // No turning motors over 90 degrees
            desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

            mDesiredVel = desiredState.speedMetersPerSecond;
            mTransMotor.set(mDesiredVel/this.mPhysicalMaxSpeedMPS);
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
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        super.setDesiredState(desiredState);
        // Stops returning to original rotation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.0001) {
            stop();
            return;
        }

        // No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // PID Controller for both translation and rotation
        mDesiredVel = desiredState.speedMetersPerSecond;
        double feedforward = mTransFF.calculate(mDesiredVel);
        double pidOutput = mTransPID.calculate(getTransVelocity(), mDesiredVel) / mPhysicalMaxSpeedMPS;
        mTransMotor.set(pidOutput + feedforward);
    }



    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the
     *         form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    @Override
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
    @Override
    public double getTransPosition() {
        return getTransPositionRaw();
    }


    /**
     * 
     * @return Returns velocity of translation motor with conversion from the CANcoder
     */
    @Override
    public double getTransVelocity() {
        return getTransVelocityRaw();
    }

    /**
     * 
     * @return the PID setpoint of the translation's velocity in meters/sec
     */
    @Override
    public double getTransVelocitySetpoint(){
        return mDesiredVel;
    }
    /**
     * 
     * @return Returns the applied voltage to the translation motor after nominal voltage
     *         compensation
     */
    @Override
    public double getTransAppliedVolts() {
        return mTransMotor.getAppliedOutput() * mTransMotor.getVoltageCompensationNominalVoltage();
    }

    /**
     * Reset ONLY the translation encoder
     */
    @Override
    public void resetEncoders() {
        mTransEncoder.setPosition(0);
    }

    /**
     * Stops the both motors
     */
    @Override
    public void stop() {
        mTransMotor.set(0);
        mRotMotor.set(0);
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
     * @return the nominal voltage amount after voltage compensation for the translation motor
     */
    public double getTransNominalVoltage(){
        return mTransMotor.getVoltageCompensationNominalVoltage();
    }
}
