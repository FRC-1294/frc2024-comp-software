// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.abstract_subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.constants.GameConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.TalonControlType;
import frc.robot.constants.LauncherConstants.LauncherState;


abstract class ShooterBase extends SubsystemBase {


  /** Creates a new ShooterBase. */
  protected static final TalonFX mLeftFlywheelMotor = new TalonFX(LauncherConstants.LEFT_FLYWHEEL_TALON_ID,"rio");
  protected static final TalonFX mRightFlywheelMotor = new TalonFX(LauncherConstants.RIGHT_FLYWHEEL_TALON_ID,"rio");

  protected double mLeftFlywheelSetpoint = LauncherConstants.LauncherState.HOME.mFlyWheelRPM;
  protected double mRightFlywheelSetpoint = LauncherConstants.LauncherState.HOME.mRollerRPM;

  public final FlywheelSim mLeftFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1),ShooterType.LEFT_FLYWHEEL.mGearRatioToSensor,LauncherConstants.LEFT_FLYWHEEL_MOI_KG_MPS,null);
  public final FlywheelSim mRightFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1),ShooterType.RIGHT_FLYWHEEL.mGearRatioToSensor, LauncherConstants.RIGHT_FLYWHEEL_MOI_KG_MPS,null);

  protected final TalonFXSimState mRollerFalconSim = mRightFlywheelMotor.getSimState();
  protected final TalonFXSimState mFlywheelFalconSim = mLeftFlywheelMotor.getSimState();

  protected final TalonFXConfiguration mRollerConfig = new TalonFXConfiguration();
  protected final TalonFXConfiguration mFlywheelConfig = new TalonFXConfiguration();

  protected final TalonFXConfigurator mFlywheelConfigurator = mLeftFlywheelMotor.getConfigurator();
  protected final TalonFXConfigurator mRollerConfigurator = mRightFlywheelMotor.getConfigurator();

  protected enum ShooterType{
    LEFT_FLYWHEEL(LauncherConstants.MOTOR_TO_RIGHT_FLYWHEEL_GEAR_RATIO, mRightFlywheelMotor),
    RIGHT_FLYWHEEL(LauncherConstants.MOTOR_TO_LEFT_FLWHEEL_GEAR_RATIO, mLeftFlywheelMotor);
    protected double mGearRatioToMotor;
    protected double mGearRatioToSensor;
    protected TalonFX mShooterObject;
    private ShooterType(double gearRatioToMotor, TalonFX shooterObject){
        mGearRatioToMotor = gearRatioToMotor;
        mGearRatioToSensor = 1/mGearRatioToMotor;
        mShooterObject = shooterObject;
    }
}

  protected TalonControlType mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
  
  protected final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0)
  .withSlot(0);

  protected final VelocityDutyCycle mDutyCycle = new VelocityDutyCycle(0)
  .withSlot(0);

  protected final CoastOut mCoastOut = new CoastOut()
  .withUpdateFreqHz(100);

  protected final VoltageOut mVoltageOut = new VoltageOut(0)
  .withUpdateFreqHz(100);

  protected ShooterBase() {
    mRightFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());
    mLeftFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());

    setOnboardFeedbackConstants();

    mLeftFlywheelMotor.setInverted(LauncherConstants.INVERT_LEFT_FLYWHEEL_MOTOR);
    mRightFlywheelMotor.setInverted(LauncherConstants.INVERT_RIGHT_FLYWHEEL_MOTOR);

    mRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    mFlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
  
  }
  protected abstract void setOnboardFeedbackConstants();

  abstract void runSpeedControl();
  /**
   * Set the desired velocity of the roller in nRPM
   * @return
   */
  abstract void setDesiredLeftFlywheelRPM(double rpm);

  /**
   * Set the desired velocity of the roller in nRPM
   * @return
   */
  abstract void setDesiredRightFlywheelRPM(double rpm);

  abstract void changeLeftFlywheelRPM(double increment);

  abstract void changeRightFlywheelRPM(double increment);

  abstract boolean atDesiredLeftFlywheelRPM();

  abstract boolean atDesiredRightFlywheelRPM();
  
  /**
   * Set the velocity of the flywheel MOTOR in native units (Rotations/Sec)
   * @return
   */
  public abstract void setFlywheelVelRaw(double rps);

  /**
   * Set the velocity of the roller MOTOR in native units (Rotations/Sec)
   * @return
   */
  public abstract void setRollerVelRaw(double rps);


  public void setToShooterState(LauncherState state){
    mControlSignal = TalonControlType.VELOCITY_VOLTAGE;
    setDesiredLeftFlywheelRPM(state.mFlyWheelRPM);
    setDesiredRightFlywheelRPM(state.mRollerRPM);
  }

  public double getRollerRPM(){
    return nativeUnitsToVelocity(mRightFlywheelMotor.getRotorVelocity().getValue(),ShooterType.RIGHT_FLYWHEEL);
  }
  /**
   * @return roller motor velocity in rotations per second of the MOTOR without gear ratio
   */
  public double getRollerVelRaw(){
    return mRightFlywheelMotor.getRotorVelocity().getValue();
  }

  public double getFlywheelVelRaw(){
    return mLeftFlywheelMotor.getRotorVelocity().getValue();
  }

  public double getFlywheelRPM(){
    return nativeUnitsToVelocity(mLeftFlywheelMotor.getRotorVelocity().getValue(), ShooterType.LEFT_FLYWHEEL);
  }
  public void writeMotorDebugData(){
    SmartDashboard.putString("Exit Pose shooter", calcExitPos().toString());
    SmartDashboard.putNumber("Exit Angle shooter", calcExitAngle());
    SmartDashboard.putNumber("Right Flywheel RPS NativeUnits", mRightFlywheelMotor.getRotorVelocity().getValue());
    SmartDashboard.putNumber("Left FlyWheel RPS NativeUnits", mLeftFlywheelMotor.getRotorVelocity().getValue());

    SmartDashboard.putNumber("FlyWheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("RollerWheel RPM", getRollerRPM());
    
    SmartDashboard.putNumber("Flywheel Setpoint RPM", mLeftFlywheelSetpoint);
    SmartDashboard.putNumber("Roller Setpoint RPM", mRightFlywheelSetpoint);

    SmartDashboard.putNumber("Native PID Output", mLeftFlywheelMotor.getClosedLoopOutput().getValue());
    SmartDashboard.putNumber("Native PID Error", mLeftFlywheelMotor.getClosedLoopError().getValue());
    SmartDashboard.putNumber("Native PID Setpoint", mLeftFlywheelMotor.getClosedLoopReference().getValue());

    SmartDashboard.putNumber("MOI",LauncherConstants.LEFT_FLYWHEEL_MOI_KG_MPS);
    SmartDashboard.putNumber("Mass",LauncherConstants.LEFT_FLYWHEEL_WEIGHT_KG);

    if (RobotBase.isSimulation()){
      SmartDashboard.putNumber("Left Flywheel sim output", mLeftFlywheelSim.getAngularVelocityRPM());
      SmartDashboard.putNumber("Right Flywheel sim output", mRightFlywheelSim.getAngularVelocityRPM());
    }

    SmartDashboard.updateValues();
  }
  abstract void writeControllerDebugData();
  
  public void stopMotors(){
    mControlSignal = TalonControlType.COAST_OUT;
    mLeftFlywheelSetpoint = 0;
    mRightFlywheelSetpoint = 0;
  }

  @Override
  public void periodic() {
    mRightFlywheelSetpoint = MathUtil.clamp(mRightFlywheelSetpoint, -LauncherConstants.PHYSICAL_MAX_RPM_RIGHT_FLYWHEEL, LauncherConstants.PHYSICAL_MAX_RPM_RIGHT_FLYWHEEL);
    mLeftFlywheelSetpoint = MathUtil.clamp(mLeftFlywheelSetpoint, -LauncherConstants.PHYSICAL_MAX_RPM_LEFT_FLYWHEEL, LauncherConstants.PHYSICAL_MAX_RPM_LEFT_FLYWHEEL);

    if(GameConstants.DEBUG_MODE){
      writeMotorDebugData();
      writeControllerDebugData();
    }
    runSpeedControl();
  }

  protected double velocityToNativeUnits(double velocityRPM, ShooterType mode){
    double motorRotationsPerMin = velocityRPM * mode.mGearRatioToSensor;
    return motorRotationsPerMin/60;
  }

  protected double nativeUnitsToVelocity(double rotationsPerSecond, ShooterType mode){
    double motorRotationsPerMin = rotationsPerSecond * 60;
    return motorRotationsPerMin * mode.mGearRatioToMotor;
  }

  private static Translation3d calcExitPos(){
    Translation3d originTranslation = LauncherConstants.RIGHT_FLYWHEEL_LOC_M.minus(LauncherConstants.LEFT_FLYWHEEL_LOC_M);
    double midpoint = (originTranslation.getNorm()-LauncherConstants.RIGHT_FLYWHEEL_RADIUS_M-LauncherConstants.LEFT_FLYWHEEL_RADIUS_M)/2;
    Translation3d midPointVector = Util.getNormalTranslation(originTranslation).times((LauncherConstants.LEFT_FLYWHEEL_RADIUS_M + midpoint));
    return midPointVector.plus(LauncherConstants.LEFT_FLYWHEEL_LOC_M);
  }

  private static double calcExitAngle(){
    Translation3d originTranslation = LauncherConstants.RIGHT_FLYWHEEL_LOC_M.minus(LauncherConstants.LEFT_FLYWHEEL_LOC_M);
    Translation3d perpendicularVec = originTranslation.rotateBy(new Rotation3d(0, Math.PI/2, 0));
    Translation3d homePerpendicularVec = perpendicularVec.minus(originTranslation);
    return Math.toDegrees(Math.atan(homePerpendicularVec.getZ()/homePerpendicularVec.getX()));
  }

  private double calcExitVel(){
    //TODO
    return -1;
  }

  public double calcTrajectory(){
    //TODO
    return -1;
  }

  @Override
  public void simulationPeriodic(){
    writeMotorDebugData();

    mFlywheelFalconSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    mRollerFalconSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    mLeftFlywheelSim.setInputVoltage(mFlywheelFalconSim.getMotorVoltage());
    mRightFlywheelSim.setInputVoltage(mRollerFalconSim.getMotorVoltage());

    mLeftFlywheelSim.update(0.02);
    mRightFlywheelSim.update(0.02);

    mFlywheelFalconSim.setRotorVelocity(velocityToNativeUnits(mLeftFlywheelSim.getAngularVelocityRPM(),ShooterType.LEFT_FLYWHEEL));
    mRollerFalconSim.setRotorVelocity(velocityToNativeUnits(mRightFlywheelSim.getAngularVelocityRPM(),ShooterType.RIGHT_FLYWHEEL));
  }
}
