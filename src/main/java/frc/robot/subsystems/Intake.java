// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax mIntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID,MotorType.kBrushless);
  //idk how to initialize beam break but you need to do it here TODO
  public Intake() {}
  public void configMotor(){
    mIntakeMotor.setSmartCurrentLimit(IntakeConstants.SMART_CURRENT_LIMIT); //set current limit as to not burn out motor
    mIntakeMotor.setInverted(IntakeConstants.INTAKE_INVERTED);
    mIntakeMotor.disableVoltageCompensation();//No voltage comp since we want intake to run at full power
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeAtSpeed(double percentOutput){
    //rotate motors to intake piece TODO
  }

  public Command getTimedIntakeCommand(double wait_time, double intake_speed){
    return new SequentialCommandGroup(new InstantCommand(()-> intakeAtSpeed(intake_speed)), new WaitCommand(wait_time));
  }

  public void getPieceInIntake(){
    //return the status of the beam break sensor to indicate if there is a piece in the intake
  }
}
