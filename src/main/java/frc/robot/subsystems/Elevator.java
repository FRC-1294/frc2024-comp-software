// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.CompConstants;
import frc.robot.constants.ElevatorConstants.AimState;


public class Elevator extends SubsystemBase {
  public enum AimingMotorMode {COAST, BRAKE}

  // Elevator Hardware
  private final TalonFX mLeftElevatorMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_TALON_ID,"rio");
  private final TalonFX mRightElevatorMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_TALON_ID,"rio");
  
  // Wrist Hardware
  // private final CANSparkMax mLeftWristMotor = new CANSparkMax(ElevatorConstants.LEFT_WRIST_SPARK_ID, MotorType.kBrushless);
  // private final CANSparkMax mRightWristMotor = new CANSparkMax(ElevatorConstants.RIGHT_WRIST_SPARK_ID, MotorType.kBrushless);
  // private final DutyCycleEncoder mWristThroughBoreEncoder = new DutyCycleEncoder(ElevatorConstants.ELEVATOR_THROUGHBORE_ENCODER_ID);

  // Current States
  private double mCurrentElevatorDistanceIn = ElevatorConstants.MIN_ELEVATOR_DIST_METERS;
  // private double mCurrentWristRotationDeg = ElevatorConstants.MIN_WRIST_ROTATION_DEG;

  // Desired States
  private double mDesiredElevatorDistanceIn = mCurrentElevatorDistanceIn;
  // private double mDesiredWristRotationDeg = mCurrentWristRotationDeg;

  // MotorMode Chooser
  private final SendableChooser<AimingMotorMode> mChooser = new SendableChooser<>();

  // PID Controllers and Motor Configs 
  PIDController mElevatorController = ElevatorConstants.mElevatorPIDConstants.toWPIController();  
  // PIDController mWristController = ElevatorConstants.mWristPIDConstants.toWPIController();

  // ArmFeedforward mWristFeedforwardController = new ArmFeedforward(ElevatorConstants.mWristPIDConstants.getKS(), ElevatorConstants.WRIST_KG, ElevatorConstants.mWristPIDConstants.getKV());

  // MotorOutputConfigs mLeftWristMotorOutputConfigs = new MotorOutputConfigs();
  MotorOutputConfigs mRightWristMotorOutputConfigs = new MotorOutputConfigs();

  Slot0Configs mElevatorControllerSlot0Configs = new Slot0Configs();

  public Elevator() {
    mChooser.addOption("Brake", AimingMotorMode.BRAKE);
    mChooser.addOption("Coast", AimingMotorMode.COAST);
    mChooser.setDefaultOption("Brake", ElevatorConstants.INITIAL_MOTOR_MODE);
  }

  // Setting Conversions and Inversions
  public void configureDevices() {

    //initialize PID Controller Constants for SlotConfigs

    mElevatorControllerSlot0Configs = ElevatorConstants.mElevatorPIDConstants.toTalonConfiguration().Slot0;

    mLeftElevatorMotor.getConfigurator().apply(mElevatorControllerSlot0Configs);
    mRightElevatorMotor.getConfigurator().apply(mElevatorControllerSlot0Configs);
    
    // mWristController = ElevatorConstants.mWristPIDConstants.toWPIController();
    //note: configuration uses internal encoders inside the motors, subject to change
  
    mLeftElevatorMotor.setInverted(true);
    // mLeftWristMotor.setInverted(true);
    // mWristController.setTolerance(ElevatorConstants.WRIST_TOLERANCE_DEG);

    //follower configuration
    // mRightWristMotor.follow(mLeftWristMotor);
    mRightElevatorMotor.setControl(new Follower(mLeftElevatorMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    updateMotorModes();
    elevatorPeriodic();
    // wristPeriodic();
    debugSmartDashboard();
  }

  private void elevatorPeriodic() {
    //Clamping Rotation between domain
    mDesiredElevatorDistanceIn = MathUtil.clamp(mDesiredElevatorDistanceIn, ElevatorConstants.MIN_ELEVATOR_DIST_METERS, ElevatorConstants.MAX_ELEVATOR_DIST_METERS);
    mCurrentElevatorDistanceIn = getCurrentElevatorDistance();

    //Temp Regular PID
    double elevatorPIDCalculation = mElevatorController.calculate(mCurrentElevatorDistanceIn, mDesiredElevatorDistanceIn);
    mLeftElevatorMotor.set(elevatorPIDCalculation + ElevatorConstants.ELEVATOR_FEEDFORWARD_CONSTANT);
  }

  // private void wristPeriodic() {
  //   //Clamping Rotation between domain
  //   mDesiredWristRotationDeg = MathUtil.clamp(mDesiredWristRotationDeg, ElevatorConstants.MIN_WRIST_ROTATION_DEG, ElevatorConstants.MAX_WRIST_ROTATION);
  //   mCurrentWristRotationDeg = getCurrentWristRotation();

  //   double offset = (Math.PI / 2);
  //   double wristPIDCalculation = mWristController.calculate(mCurrentWristRotationDeg, mDesiredWristRotationDeg);    
  //   double wristFeedforwardCalculation = mWristFeedforwardController.calculate(Math.toRadians(mDesiredWristRotationDeg) - offset, mLeftWristMotor.getEncoder().getVelocity() * ElevatorConstants.SPARK_THROUGHBORE_GEAR_RATIO);
  //   mLeftWristMotor.set(wristPIDCalculation + wristFeedforwardCalculation);
  // }

  private void updateMotorModes() {
    SmartDashboard.updateValues();

    AimingMotorMode mode = mChooser.getSelected();
    
    // Motors go towards setpoints
    NeutralModeValue neutralmode;
    IdleMode idleMode;

    switch (mode) {
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

    // mRightWristMotor.setIdleMode(idleMode);
    // mLeftWristMotor.setIdleMode(idleMode);
  }

  // Contains Smart Dashboard Statements ONLY ON DEBUG
  private void debugSmartDashboard() {
    if (CompConstants.DEBUG_MODE) {
      // SmartDashboard.putNumber("Current Wrist Rotation", mCurrentWristRotationDeg);
      SmartDashboard.putNumber("Current Elevator Distance", mCurrentElevatorDistanceIn);
      // SmartDashboard.putNumber("Desired Wrist Rotation", mDesiredWristRotationDeg);
      SmartDashboard.putNumber("Desired Elevator Distance", getDesiredElevatorDistance());
      SmartDashboard.putNumber("Raw Elevator Encoder Position", mLeftElevatorMotor.getPosition().getValueAsDouble());
      // SmartDashboard.putNumber("Raw Wrist Encoder Rotation", mWristThroughBoreEncoder.getAbsolutePosition());
      SmartDashboard.putBoolean("At Elevator setpoint", atElevatorSetpoint());
      // SmartDashboard.putBoolean("At Wrist setpoint", atWristSetpoint());
    }
  }

  // Getters and Setters of States
  public double getCurrentElevatorDistance() {
    return mLeftElevatorMotor.getRotorPosition().getValueAsDouble() * ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS;
  }

  /**
   * 
   * @return Current Wrist Rotation in Degrees
   */
  // public double getCurrentWristRotation(){
  //   return mWristThroughBoreEncoder.getAbsolutePosition();
  // }

  public double getDesiredElevatorDistance() {
    return mDesiredElevatorDistanceIn;
  }
  
  // public double getDesiredWristRotation() {
  //   return mDesiredWristRotationDeg;
  // }

  public void setDesiredElevatorDistance(double distance) {
    mDesiredElevatorDistanceIn = distance;
  }

  // public void setDesiredWristRotation(double rotation) {
  //   mDesiredWristRotationDeg = rotation;
  // }

  public void setDesiredSetpoint(AimState state) {
    mDesiredElevatorDistanceIn = state.elevatorDistIn;
    // mDesiredWristRotationDeg = state.wristAngleDeg;
  }
  public void changeDesiredElevatorPosition(double increment) {
    mDesiredElevatorDistanceIn += increment;
  }

  // public void changeDesiredWristRotation(double increment) {
  //   mDesiredWristRotationDeg += increment;
  // }

  public boolean atElevatorSetpoint() {
    // ENCODER VERSION
    // return mLeftElevatorMotor.getRotorPosition().getValueAsDouble() * AimingConstants.ELEVATOR_ROTATIONS_TO_INCHES) < AimingConstants.ELEVATOR_TOLERANCE_IN
    return Math.abs(mDesiredElevatorDistanceIn - mCurrentElevatorDistanceIn) <= ElevatorConstants.ELEVATOR_TOLERANCE_METERS;
  }
  // public boolean atWristSetpoint() {
  //   return mWristController.atSetpoint();
  // }

  public boolean atSetpoints() {
    return atElevatorSetpoint();
  }

  public Command waitUntilSetpoint(AimState state) {
    return new FunctionalCommand(() -> setDesiredSetpoint(state), null, null, this::atSetpoints, this);  
  }

  public Command waitUntilElevatorSetpoint(double sp) {
    return new FunctionalCommand(() -> setDesiredElevatorDistance(sp), null, null, this::atElevatorSetpoint, this);  
  }

  // public Command waitUntilWristSetpoint(double sp) {
  //   return new FunctionalCommand(() -> setDesiredWristRotation(sp), null, null, this::atWristSetpoint, this);  
  // }
}