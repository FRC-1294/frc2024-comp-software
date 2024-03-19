// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax mIntakeMotorInner = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID_INNER, MotorType.kBrushless);
  private final CANSparkMax mIntakeMotorOuter = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID_OUTER, MotorType.kBrushless);
  private static final DigitalInput mBeamBreak = new DigitalInput(IntakeConstants.INTAKE_BEAMBREAK_ID);
  private boolean beamBreakOverride = false;

  public IntakeSubsystem() {
    mIntakeMotorInner.restoreFactoryDefaults();
    mIntakeMotorOuter.restoreFactoryDefaults();
    // mIntakeMotorInner.setSmartCurrentLimit(40); //set current limit as to not burn out motor
    mIntakeMotorInner.setInverted(IntakeConstants.INTAKE_INVERTED_INNER);
    mIntakeMotorInner.enableVoltageCompensation(11);//No voltage comp since we want intake to run at full power
 
    mIntakeMotorOuter.setSmartCurrentLimit(40); //set current limit as to not burn out motor
    mIntakeMotorOuter.setInverted(IntakeConstants.INTAKE_INVERTED_OUTER);
    mIntakeMotorOuter.enableVoltageCompensation(11);//No voltage comp since we want intake to run at full power


    mIntakeMotorInner.setIdleMode(IdleMode.kBrake);
    mIntakeMotorOuter.setIdleMode(IdleMode.kBrake);

    mIntakeMotorInner.burnFlash();
    mIntakeMotorOuter.burnFlash();
  }

  @Override
  public void periodic() {
    // if (pieceInIntake() && !beamBreakOverride){
    //   stopMotors();
    // }
    SmartDashboard.putBoolean("Piece in Intake", pieceInIntake());
  }

  /**
   * Running the Motors with no PID
   * @param percentOutput Percent to running (from -1.0 to 1.0)
   */
  public void intakeMotorsAtSpeed(double percentOutput){
    mIntakeMotorInner.set(percentOutput);
    mIntakeMotorOuter.set(percentOutput);
  }

  public void innerMotorAtSpeed(double percentOutput){
    mIntakeMotorInner.set(percentOutput);
  }

  public void outerMotorAtSpeed(double percentOutput){
    mIntakeMotorOuter.set(percentOutput);
  }

  public void runIntakeMotors(){
    mIntakeMotorInner.set(IntakeConstants.INNER_INTAKE_SPEED_AQUIRE);
    mIntakeMotorOuter.set(IntakeConstants.OUTER_INTAKE_SPEED_ACTIVE);
  }

  public void stopMotors() {
    mIntakeMotorInner.set(0.0);
    mIntakeMotorOuter.set(0.0);
  }

  public Command getTimedIntakeCommand(double waitTime, double intakeSpeed){
    return new SequentialCommandGroup(new InstantCommand(()-> intakeMotorsAtSpeed(intakeSpeed)), new WaitCommand(waitTime));
  }

  public Command getAutomousIntakeCommand() {
    return new FunctionalCommand(() -> intakeMotorsAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_AQUIRE), 
    ()->{}, interrupted -> stopMotors(), ()->IntakeSubsystem.pieceInIntake(), this);
  }

  public static boolean pieceInIntake(){
    return !mBeamBreak.get();
  }
  public boolean toggleBeamBreakOverride(){
    beamBreakOverride = !beamBreakOverride;
    return beamBreakOverride;
  }
}
