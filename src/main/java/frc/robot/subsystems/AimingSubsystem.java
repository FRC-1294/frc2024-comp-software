// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;


public class AimingSubsystem extends SubsystemBase {
  public enum AimingMotorMode {COAST, BRAKE, OFF}

  // Elevator Hardware
  private final TalonFX mLeftFlywheelMotor = new TalonFX(AimingConstants.LEFT_ELEVATOR_TALON_ID,"rio");
  private final TalonFX mRightFlywheelMotor = new TalonFX(AimingConstants.RIGHT_ELEVATOR_TALON_ID,"rio");

  // TOF
  private final TimeOfFlight mElevatorTOF = new TimeOfFlight(AimingConstants.ELEVATOR_TOF_ID);
  
  // Wrist Hardware
  private final CANSparkMax mWristMotor = new CANSparkMax(AimingConstants.WRIST_SPARK_ID, MotorType.kBrushless);
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
  private final PIDController mElevatorController = AimingConstants.mElevatorPIDConstants.toWPIController();
  private final PIDController mWristController = AimingConstants.mWristPIDConstants.toWPIController();



  public AimingSubsystem() {
    mChooser.addOption("Brake", AimingMotorMode.BRAKE);
    mChooser.addOption("Coast", AimingMotorMode.COAST);
    mChooser.setDefaultOption("Brake", AimingConstants.INITIAL_MOTOR_MODE);

    mElevatorController.setTolerance(AimingConstants.ELEVATOR_TOLERANCE_IN);
    mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);

    //Initializes TOF Sensor
    setTOFViewZone(8, 8, 12, 12);
    setTOFRange("short", 20);

    configureDevices();
  }

  // Setting Conversions and Inversions
  public void configureDevices() {
    //TODO: Set the Converstions and Inversions

  }

  @Override
  public void periodic() {
    elevatorPeriodic();
    wristPeriodic();
    //TODO: Implement This
  }

  private void elevatorPeriodic() {
      //TODO: Add Clamps, PID, and Run Motor
  }

  private void wristPeriodic() {
    //TODO: Add Clamps, PID, and Run Motor
  }

  private void updateMotorModes() {
    SmartDashboard.updateValues();

    AimingMotorMode mode = mChooser.getSelected();
    //TODO: Logic
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
  public double getDesiredElevatorDistance() {
    return mDesiredElevatorDistanceIn;
  }
  
  public double getDesiredWristRotation() {
    return mDesiredWristRotationDeg;
  }

  public double getCurrentElevatorDistance() {
    return mCurrentElevatorDistanceIn;
  }

  public void setDesiredElevatorDistance(double distance) {
    mDesiredElevatorDistanceIn = distance;
  }

  public void setDesiredSetpoint(AimState state) {
    mDesiredElevatorDistanceIn = state.elevatorDistIn;
    mDesiredWristRotationDeg = state.wristAngleDeg;
  }

  public void setDesiredWristRotation(double rotation) {
    mDesiredWristRotationDeg = rotation;
  }

  public boolean atElevatorSetpoint(){return mElevatorController.atSetpoint();}
  public boolean atWristSetpoint() {return mWristController.atSetpoint();}
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

