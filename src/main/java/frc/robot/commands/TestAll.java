// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TestAll extends Command {
  IntakeSubsystem intake;
  AimingSubsystem aiming;
  LauncherSubsystem launcher;
  SwerveSubsystem swerve;

  public TestAll(IntakeSubsystem intake, AimingSubsystem aiming, LauncherSubsystem launcher, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.aiming = aiming;
    this.launcher = launcher;
    this.swerve = swerve;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testIntake();
    testAim();
    testLauncher();
    testSwerve();
  }

  public void testIntake() {
    intake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
    new WaitUntilCommand(intake::pieceInIntake);
    intake.stopMotors();
  }

  public void testAim() {
    aiming.changeDesiredWristRotation(90);
    new WaitUntilCommand(aiming::atWristSetpoint);
    aiming.changeDesiredWristRotation(-90);
    aiming.changeDesiredElevatorPosition(5);
    new WaitUntilCommand(aiming::atElevatorSetpoint);
    aiming.changeDesiredElevatorPosition(-5);
  }

  public void testLauncher() {
    launcher.runLauncher();
    while (true) {
      if (!launcher.pieceInIndexer()) {
        launcher.stopLauncher();
        break;
      }
    } 
  }

  public void testSwerve() {
    swerve.setChassisSpeed(1, 1, 1, false);
    new WaitCommand(5);
    swerve.setChassisSpeed(0,0,0,false);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}