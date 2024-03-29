package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Util.PIDParameters;
import frc.robot.Util.TalonControlType;

public class KrakenSwerveModule extends SwerveModuleAbstract{

    // Hardware
    // Motor Controllers
    private final TalonFX mTransMotor;
    // Encoders
    private final TalonFXConfiguration mTransConfiguration;
    private final VelocityVoltage mVelocityVoltageSignal = new VelocityVoltage(0).withSlot(1);
    private final CoastOut mCoastOutSignal = new CoastOut();
    private final double mNominalVoltage;

    // Public Debugging Values
    private double mMaxAccel = 0.0;
    private double mCurAccel = 0.0;


    public KrakenSwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse,
            boolean transInverse, PIDParameters rotPID, PIDParameters transPID, double transGearRatio,
            double wheelCircumference, double physicalMaxSpeed, double absEncGearRatio, double relEncoderGearRatio) {
        // Setting Parameters

        super(rotID, transID, rotEncoderID, rotInverse, transInverse, rotPID, transPID, transGearRatio, wheelCircumference, physicalMaxSpeed, absEncGearRatio, relEncoderGearRatio);
        mNominalVoltage = 12;
 

        // ----Setting Hardware
        // Motor Controllers
        mTransMotor = new TalonFX(transID,"DriveMotors");
        mTransConfiguration = new TalonFXConfiguration();

        mTransConfiguration.Feedback.SensorToMechanismRatio = 1/(transGearRatio *wheelCircumference);
        mTransConfiguration.withSlot0(transPID.toTalonConfiguration());
        mTransMotor.getConfigurator().apply(mTransConfiguration);



        // ----Setting PID Parameters
        // ----Setting Inversion
        mTransMotor.setInverted(mTransInverse);

        mTransMotor.setNeutralMode(NeutralModeValue.Brake);

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
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTransVelocity(), Rotation2d.fromRadians(getRotPosition()));
    }


    /**
     * 
     * Sets the translation motor's voltage (max 12 volts)
     */
    @Override
    public void setTranslationVoltageRaw(double volts) {
        mTransMotor.setVoltage(volts);
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
        return mTransMotor.getPosition().getValueAsDouble();
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
    @Override
    public double getTransAppliedVolts() {
        return mTransMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setTransMotorDutyCycle(double speed){
         mTransMotor.setVoltage(speed*mNominalVoltage);
    }

    /**
     * Reset ONLY the translation encoder
     */
    @Override
    public void resetEncoders() {
        mTransMotor.setPosition(0);
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
    public void setModeTrans(TalonControlType mode) {
        switch (mode) {
            case COAST_OUT:
                mTransMotor.setControl(mCoastOutSignal);
                break;
            case VELOCITY_VOLTAGE:
                mTransMotor.setControl(mVelocityVoltageSignal);
                break;
            default:
                break;
        }
    }

    /**
     * Permanently burns settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        mRotMotor.burnFlash();
    }

    /**
     * @return the nominal voltage amount after voltage compensation for the translation motor
     */
    @Override
    public double getTransNominalVoltage(){
        return mNominalVoltage; 
    }
}
