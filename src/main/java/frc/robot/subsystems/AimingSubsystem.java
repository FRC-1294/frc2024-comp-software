// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;


public class AimingSubsystem extends SubsystemBase {
  public enum AimingMotorMode {COAST, BRAKE}

  // Elevator Hardware
  private final TalonFX mLeftFlywheelMotor = new TalonFX(AimingConstants.LEFT_ELEVATOR_TALON_ID,"rio");
  private final TalonFX mRightFlywheelMotor = new TalonFX(AimingConstants.RIGHT_ELEVATOR_TALON_ID,"rio");

  // TOF
  private final TimeOfFlight mElevatorTOF = new TimeOfFlight(AimingConstants.ELEVATOR_TOF_ID);
  
  // Wrist Hardware
  private final TalonFX mWristMotor = new TalonFX(AimingConstants.WRIST_SPARK_ID, "rio");
  private final CANcoder mWristEncoder = new CANcoder(AimingConstants.WRIST_ENCODER_ID);

  // Current States
  private double mCurrentElevatorDistanceIn = AimingConstants.MIN_ELEVATOR_DIST_IN;
  private double mCurrentWristRotationDeg = AimingConstants.MIN_WRIST_ROTATION_DEG;

  // Desired States
  private double mDesiredElevatorDistanceIn = mCurrentElevatorDistanceIn;
  private double mDesiredWristRotationDeg = mCurrentWristRotationDeg;

  // MotorMode Chooser
  private final SendableChooser<AimingMotorMode> mChooser = new SendableChooser<>();

  // PID Controllers
  Slot0Configs mElevatorController = new Slot0Configs();
  Slot0Configs mWristController = new Slot0Configs();  

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
    //TODO: Set the Converstions and Inversions

    //initialize PID Controller Constants
    mElevatorController.kP = AimingConstants.mElevatorPIDConstants.mKP;
    mElevatorController.kI = AimingConstants.mElevatorPIDConstants.mKI;
    mElevatorController.kD = AimingConstants.mElevatorPIDConstants.mKD;

    mWristController.kP = AimingConstants.mWristPIDConstants.mKP;
    mWristController.kI = AimingConstants.mWristPIDConstants.mKI;
    mWristController.kD = AimingConstants.mWristPIDConstants.mKD;

    mLeftFlywheelMotor.getConfigurator().apply(mElevatorController);
    mRightFlywheelMotor.getConfigurator().apply(mElevatorController);
    mWristMotor.getConfigurator().apply(mWristController);

    mLeftFlywheelMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    updateMotorModes();
    elevatorPeriodic();
    wristPeriodic();
  }

  private void elevatorPeriodic() {
    //TODO: Add Clamps, PID, and Run Motor

    PositionVoltage elevatorVoltage = new PositionVoltage(0).withSlot(0);
    mLeftFlywheelMotor.setControl(elevatorVoltage.withPosition(getDesiredElevatorDistance()));
    mRightFlywheelMotor.setControl(elevatorVoltage.withPosition(getDesiredElevatorDistance()));

    mCurrentElevatorDistanceIn = mLeftFlywheelMotor.getRotorPosition().getValueAsDouble() * AimingConstants.ELEVATOR_ROTATIONS_TO_INCHES;
  }

  private void wristPeriodic() {
    //TODO: Add Clamps, PID, and Run Motor

    PositionVoltage wristVoltage = new PositionVoltage(0).withSlot(0);
    mWristMotor.setControl(wristVoltage.withPosition(getDesiredWristRotation()));
    mCurrentWristRotationDeg = mWristMotor.getRotorPosition().getValueAsDouble() * 360;
  }

  private void updateMotorModes() {
    SmartDashboard.updateValues();

    AimingMotorMode mode = mChooser.getSelected();
    
    // Motors go towards setpoints
    NeutralModeValue neutralmode = mode == AimingMotorMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    mLeftFlywheelMotor.setNeutralMode(neutralmode);
    mRightFlywheelMotor.setNeutralMode(neutralmode);
    mWristMotor.setNeutralMode(neutralmode);

  }

  // Contains Smart Dashboard Statements ONLY ON DEBUG
  private void debugSmartDashboard() {

    // TODO: Add debug statements
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
    return mCurrentElevatorDistanceIn;
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
  public void changeDesiredElevatorPosition(double rate) {
    mDesiredElevatorDistanceIn += rate;
  }

  public void changeDesiredWristRotation(double rate) {
    mDesiredWristRotationDeg += rate;
  }

  public boolean atElevatorSetpoint() {

    // ENCODER VERSION
    // if (mLeftFlywheelMotor.getRotorPosition().getValueAsDouble() * AimingConstants.ELEVATOR_ROTATIONS_TO_INCHES) < AimingConstants.ELEVATOR_TOLERANCE_IN) {
    //   return true;
    // }
 
    if (Math.abs(AimingConstants.MAX_ELEVATOR_DIST - getTOFRangeInches()) <= AimingConstants.ELEVATOR_TOLERANCE_IN) {
      return true;
    }
    return false;
  }
  public boolean atWristSetpoint() {
    if (Math.abs(mWristEncoder.getPosition().getValueAsDouble() - mWristMotor.getRotorPosition().getValueAsDouble()) * 360 <= AimingConstants.WRIST_TOLERANCE_DEG) {
      return true;
    }

    return false;
  }
  public boolean atSetpoints() {return atElevatorSetpoint() && atWristSetpoint();}

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

