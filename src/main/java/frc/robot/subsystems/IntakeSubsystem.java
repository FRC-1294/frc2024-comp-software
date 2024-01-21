// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax mIntakeMotor;
  private final DigitalInput mBeamBreak;

  public IntakeSubsystem() {
    mIntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID,MotorType.kBrushless);
    mIntakeMotor.setSmartCurrentLimit(IntakeConstants.SMART_CURRENT_LIMIT); //set current limit as to not burn out motor
    mIntakeMotor.setInverted(IntakeConstants.INTAKE_INVERTED);
    mIntakeMotor.disableVoltageCompensation();//No voltage comp since we want intake to run at full power

    mBeamBreak = new DigitalInput(IntakeConstants.INTAKE_BEAMBREAK_ID);
  }

  @Override
  public void periodic() {}

  /**
   * Running the Motors with no PID
   * @param percentOutput Percent to running (from -100 to 100)
   */
  public void intakeAtSpeed(double percentOutput){
    mIntakeMotor.set(percentOutput/100);
  }

  public void stopMotor() {
    mIntakeMotor.set(0.0);
  }

  public Command getTimedIntakeCommand(double wait_time, double intake_speed){
    return new SequentialCommandGroup(new InstantCommand(()-> intakeAtSpeed(intake_speed)), new WaitCommand(wait_time));
  }

  public boolean pieceInIntake(){
    return !mBeamBreak.get();
  }
}
