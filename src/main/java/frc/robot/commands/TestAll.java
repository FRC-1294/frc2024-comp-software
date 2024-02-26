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
  IntakeSubsystem mIntake;
  AimingSubsystem mAiming;
  LauncherSubsystem mLauncher;
  SwerveSubsystem mSwerve;
  boolean mTestIntakeSeperately;
  boolean mTestAimingSeperately;
  boolean mTestLauncherSeperately;
  boolean mTestSwerveSeperately;

  public TestAll(IntakeSubsystem intake, AimingSubsystem aiming, LauncherSubsystem launcher, SwerveSubsystem swerve,
  boolean testIntakeSeperately, boolean testAimingSeperately, boolean testLauncherSeperately, boolean testSwerveSeperately) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mAiming = aiming;
    mLauncher = launcher;
    mSwerve = swerve;

    mTestIntakeSeperately = testIntakeSeperately;
    mTestAimingSeperately = testAimingSeperately;
    mTestLauncherSeperately = testLauncherSeperately;
    mTestSwerveSeperately = testSwerveSeperately;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mTestIntakeSeperately) {
      testIntake();
    }
    if (mTestAimingSeperately) {
      testAim();
    }
    if (mTestLauncherSeperately) {
      testLauncher();
    }
    if (mTestSwerveSeperately) {
      testSwerve();
    }
    testCycle();
  }

  public void testCycle() {
    // Intake Portion
    mSwerve.setChassisSpeed(1, 0, 0, false,false); 
    mIntake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
    new WaitUntilCommand(mIntake::pieceInIntake);
    mIntake.stopMotors();
    mSwerve.setChassisSpeed(0, 0, 0, false);

    // Handoff Portion
    mAiming.setDesiredElevatorDistance(0);
    mAiming.setDesiredWristRotation(0); 
    mLauncher.runLauncher();
    new WaitCommand(1);
    mIntake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
    new WaitUntilCommand(mLauncher::pieceInIndexer);
    mLauncher.stopLauncher();
    mIntake.stopMotors();

    // Aiming and Laucher Portion
    mAiming.changeDesiredElevatorPosition(5);
    mAiming.changeDesiredWristRotation(45);
    new WaitUntilCommand(mAiming::atElevatorSetpoint);
    new WaitUntilCommand(mAiming::atWristSetpoint);
    mLauncher.runLauncher();
    new WaitCommand(2);
    
    // Reset Aiming to a normal position
    mAiming.setDesiredElevatorDistance(0);
    mAiming.setDesiredWristRotation(0); 
  }

  public void testIntake() {
    mIntake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
    new WaitCommand(5);
    mIntake.stopMotors();
  }

  public void testAim() {
    mAiming.changeDesiredWristRotation(90);
    new WaitUntilCommand(mAiming::atWristSetpoint);
    mAiming.changeDesiredWristRotation(-90);
    mAiming.changeDesiredElevatorPosition(5);
    new WaitUntilCommand(mAiming::atElevatorSetpoint);
    mAiming.changeDesiredElevatorPosition(-5);
  }

  public void testLauncher() {
    mLauncher.runLauncher();
    new WaitCommand(5);
    mLauncher.stopLauncher();
  }

  public void testSwerve() {
    mSwerve.setChassisSpeed(1, 1, 1, false);
    new WaitCommand(5);
    mSwerve.setChassisSpeed(0,0,0,false);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // only gets executed once, so no need
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // dont need
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}