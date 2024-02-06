// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.CompConstants;
import frc.robot.constants.AimingConstants.AimState;


public class AimingSubsystem extends SubsystemBase {
  public enum AimingMotorMode {COAST, BRAKE}

  // Elevator Hardware
  private final TalonFX mLeftElevatorMotor = new TalonFX(AimingConstants.LEFT_ELEVATOR_TALON_ID,"rio");
  private final TalonFX mRightElevatorMotor = new TalonFX(AimingConstants.RIGHT_ELEVATOR_TALON_ID,"rio");
  
  // TOF
  private final TimeOfFlight mElevatorTOF = new TimeOfFlight(AimingConstants.ELEVATOR_TOF_ID);
  
  // Wrist Hardware
  private final CANSparkMax mLeftWristMotor = new CANSparkMax(AimingConstants.LEFT_WRIST_SPARK_ID, MotorType.kBrushless);
  private final CANSparkMax mRightWristMotor = new CANSparkMax(AimingConstants.RIGHT_WRIST_SPARK_ID, MotorType.kBrushless);
  private final DutyCycleEncoder mWristThroughBoreEncoder = new DutyCycleEncoder(AimingConstants.ELEVATOR_THROUGHBORE_ENCODER_ID);

  // Current States
  private double mCurrentElevatorDistanceIn = AimingConstants.MIN_ELEVATOR_DIST_IN;
  private double mCurrentWristRotationDeg = AimingConstants.MIN_WRIST_ROTATION_DEG;

  // Desired States
  private double mDesiredElevatorDistanceIn = mCurrentElevatorDistanceIn;
  private double mDesiredWristRotationDeg = mCurrentWristRotationDeg;

  // MotorMode Chooser
  private final SendableChooser<AimingMotorMode> mChooser = new SendableChooser<>();

  // PID Controllers and Motor Configs 
  PIDController mElevatorController = AimingConstants.mElevatorPIDConstants.toWPIController();  
  PIDController mWristController = AimingConstants.mWristPIDConstants.toWPIController();

  ArmFeedforward mWristFeedforwardController = new ArmFeedforward(AimingConstants.mWristPIDConstants.mKS, AimingConstants.mWristkG, AimingConstants.mWristPIDConstants.mKS);
  ElevatorFeedforward mElevatorFeedforward = new ElevatorFeedforward(AimingConstants.mElevatorPIDConstants.mKS, AimingConstants.mElevatorkG, AimingConstants.mElevatorPIDConstants.mKV);

  MotorOutputConfigs mLeftWristMotorOutputConfigs = new MotorOutputConfigs();
  MotorOutputConfigs mRightWristMotorOutputConfigs = new MotorOutputConfigs();

  Slot0Configs mElevatorControllerSlot0Configs = new Slot0Configs();

  // Onboard PID
  // PositionVoltage elevatorVoltage = new PositionVoltage(getDesiredElevatorDistance()).withSlot(0);

  public AimingSubsystem() {
    mChooser.addOption("Brake", AimingMotorMode.BRAKE);
    mChooser.addOption("Coast", AimingMotorMode.COAST);
    mChooser.setDefaultOption("Brake", AimingConstants.INITIAL_MOTOR_MODE);

    //Initializes TOF Sensor
    setTOFViewZone(8, 8, 12, 12);
    setTOFRange("short", 20);
    configureDevices();
  }

  // Setting Conversions and Inversions
  public void configureDevices() {

    //initialize PID Controller Constants
    mElevatorController.setPID(AimingConstants.mElevatorPIDConstants.mKP, AimingConstants.mElevatorPIDConstants.mKI, AimingConstants.mElevatorPIDConstants.mKD);

    mElevatorControllerSlot0Configs.kP = AimingConstants.mElevatorPIDConstants.mKP;
    mElevatorControllerSlot0Configs.kI = AimingConstants.mElevatorPIDConstants.mKI;
    mElevatorControllerSlot0Configs.kD = AimingConstants.mElevatorPIDConstants.mKD;
    mElevatorControllerSlot0Configs.kS = AimingConstants.mElevatorPIDConstants.mKS;
    mElevatorControllerSlot0Configs.kV = AimingConstants.mElevatorPIDConstants.mKV;

    mLeftElevatorMotor.getConfigurator().apply(mElevatorControllerSlot0Configs);
    mRightElevatorMotor.getConfigurator().apply(mElevatorControllerSlot0Configs);
    
    mWristController = AimingConstants.mWristPIDConstants.toWPIController();
    //note: configuration uses internal encoders inside the motors, subject to change
  
    mLeftElevatorMotor.setInverted(true);
    mLeftWristMotor.setInverted(true);
    mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);

    //follower configuration
    mRightWristMotor.follow(mLeftWristMotor);
    mRightElevatorMotor.setControl(new Follower(mLeftElevatorMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    updateMotorModes();
    elevatorPeriodic();
    wristPeriodic();
  }

  private void elevatorPeriodic() {
    //Clamping Rotation between domain
    mDesiredElevatorDistanceIn = MathUtil.clamp(mDesiredElevatorDistanceIn, AimingConstants.MIN_ELEVATOR_DIST_IN, AimingConstants.MAX_ELEVATOR_DIST);
    mCurrentElevatorDistanceIn = getCurrentElevatorDistance();

    //Onboard PID
    //mLeftElevatorMotor.setControl(elevatorVoltage.withPosition(mDesiredElevatorDistanceIn));

    //Temp Regular PID
    double elevatorPIDCalculation = mElevatorController.calculate(mCurrentElevatorDistanceIn, mDesiredElevatorDistanceIn);
    mLeftElevatorMotor.set(elevatorPIDCalculation);
  }

  private void wristPeriodic() {
    //Clamping Rotation between domain
    mDesiredWristRotationDeg = MathUtil.clamp(mDesiredWristRotationDeg, AimingConstants.MIN_WRIST_ROTATION_DEG, AimingConstants.MAX_WRIST_ROTATION);
    mCurrentWristRotationDeg = getCurrentWristRotation();

    double offset = (Math.PI / 2);
    double wristPIDCalculation = mWristController.calculate(mCurrentWristRotationDeg, mDesiredWristRotationDeg);    
    double wristFFCalculation = mWristFeedforwardController.calculate(Math.toRadians(mDesiredWristRotationDeg) - offset, mLeftWristMotor.getEncoder().getVelocity() * AimingConstants.SPARK_THROUGHBORE_GEAR_RATIO);
    mLeftWristMotor.set(wristPIDCalculation + wristFFCalculation);
  }

  private void updateMotorModes() {
    SmartDashboard.updateValues();

    AimingMotorMode mode = mChooser.getSelected();
    
    // Motors go towards setpoints
    NeutralModeValue neutralmode;
    IdleMode idleMode;

    switch (mode) {
        case BRAKE:
            neutralmode = NeutralModeValue.Brake;
            idleMode = IdleMode.kBrake;
            break;
        case COAST:
            neutralmode = NeutralModeValue.Coast;
            idleMode = IdleMode.kCoast;
            break;
        default:
            neutralmode = NeutralModeValue.Brake;
            idleMode = IdleMode.kBrake;
            break;
    }

    mLeftElevatorMotor.setNeutralMode(neutralmode);
    mRightElevatorMotor.setNeutralMode(neutralmode);

    mRightWristMotor.setIdleMode(idleMode);
    mLeftWristMotor.setIdleMode(idleMode);
  }

  // Contains Smart Dashboard Statements ONLY ON DEBUG
  private void debugSmartDashboard() {
    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("Current Wrist Rotation", mCurrentWristRotationDeg);
      SmartDashboard.putNumber("Current Elevator Distance", mCurrentElevatorDistanceIn);
    }
  }

  // Getters and Setters of TOF Sensor
  /**
   * 
   * @return range in millimeters
   */
  public double getTOFRangeRaw() {
    return mElevatorTOF.getRange() - 30.0;
  }

  public double getTOFRangeCentimeters() {
    return getTOFRangeRaw()/10;
  }

  public double getTOFRangeInches() {
    return Math.round(((getTOFRangeRaw())/(25.4))*100.0)/100.0;
  }

  public void setTOFViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    mElevatorTOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  public void setTOFRange(String sMode, double sampleTime) {
    switch(sMode.toLowerCase()) {
      case "short": //Use short only if less than 1.3 meters off the ground and bright light
        mElevatorTOF.setRangingMode(RangingMode.Short, sampleTime);
      case "long": //Use for dark lighting up to 4 meters
        mElevatorTOF.setRangingMode(RangingMode.Long, sampleTime);
      case "medium": //Use medium if there could potentially be dark lighting
        mElevatorTOF.setRangingMode(RangingMode.Medium, sampleTime);

    }
  }

  // Getters and Setters of States
  public double getCurrentElevatorDistance() {
    return mLeftElevatorMotor.getRotorPosition().getValueAsDouble() * AimingConstants.ELEVATOR_ROTATIONS_TO_INCHES;
  }

  /**
   * 
   * @return Current Wrist Rotation in Degrees
   */
  public double getCurrentWristRotation(){
    return mWristThroughBoreEncoder.getAbsolutePosition();
  }

  public double getDesiredElevatorDistance() {
    return mDesiredElevatorDistanceIn;
  }
  
  public double getDesiredWristRotation() {
    return mDesiredWristRotationDeg;
  }

  public void setDesiredElevatorDistance(double distance) {
    mDesiredElevatorDistanceIn = distance;
  }

  public void setDesiredWristRotation(double rotation) {
    mDesiredWristRotationDeg = rotation;
  }

  public void setDesiredSetpoint(AimState state) {
    mDesiredElevatorDistanceIn = state.elevatorDistIn;
    mDesiredWristRotationDeg = state.wristAngleDeg;
  }
  public void changeDesiredElevatorPosition(double increment) {
    mDesiredElevatorDistanceIn += increment;
  }

  public void changeDesiredWristRotation(double increment) {
    mDesiredWristRotationDeg += increment;
  }

  public boolean atElevatorSetpoint() {
    // ENCODER VERSION
    // return mLeftElevatorMotor.getRotorPosition().getValueAsDouble() * AimingConstants.ELEVATOR_ROTATIONS_TO_INCHES) < AimingConstants.ELEVATOR_TOLERANCE_IN
    return Math.abs(mDesiredElevatorDistanceIn - mCurrentElevatorDistanceIn) <= AimingConstants.ELEVATOR_TOLERANCE_IN;
  }
  public boolean atWristSetpoint() {
    return mWristController.atSetpoint();
  }

  public boolean atSetpoints() {
    return atElevatorSetpoint() && atWristSetpoint();
  }

  public Command waitUntilSetpoint(AimState state) {
    return new FunctionalCommand(() -> setDesiredSetpoint(state), null, null, this::atSetpoints, this);  
  }

  public Command waitUntilElevatorSetpoint(double sp) {
    return new FunctionalCommand(() -> setDesiredElevatorDistance(sp), null, null, this::atElevatorSetpoint, this);  
  }

  public Command waitUntilWristSetpoint(double sp) {
    return new FunctionalCommand(() -> setDesiredWristRotation(sp), null, null, this::atWristSetpoint, this);  
  }
}