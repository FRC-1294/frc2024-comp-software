// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TestAllSubsystems extends Command {
  /** Creates a new TestAllSubsystems. */
  IntakeSubsystem mIntake;
  AimingSubsystem mAiming;
  LauncherSubsystem mLauncher;
  SwerveSubsystem mSwerve;
  int mCase_num = 0; 
  SendableChooser<Integer> mChoose = new SendableChooser<>();
public TestAllSubsystems(IntakeSubsystem intake, AimingSubsystem aiming, LauncherSubsystem launcher, SwerveSubsystem swerve)
{
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mAiming = aiming;
    mLauncher = launcher;
    mSwerve = swerve;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putData("pick the test", mChoose);
    mChoose.addOption("swerve and intake", 0);
    mChoose.addOption("intake and laucher", 1);
    mChoose.addOption("wrist", 2);
    mChoose.addOption("elevator", 3);
    mChoose.addOption("launcher", 4);
  }

  // Called every time the scheduler runs which the command is scheduled.
  @Override
  public void execute() {
    mCase_num = mChoose.getSelected();
    switch (mCase_num) {
      case 0:
        mSwerve.setChassisSpeed(0.5, 0, 0, false); 
        mIntake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
        if (mIntake.pieceInIntake()) {
          mSwerve.setChassisSpeed(0,0,0,false);
          mIntake.stopMotors();
        }
        SmartDashboard.putNumber("OdoX", SwerveSubsystem.getRobotPose().getX());
        SmartDashboard.putNumber("OdoY", SwerveSubsystem.getRobotPose().getY());
        SmartDashboard.putNumber("RobotRotDeg", SwerveSubsystem.getHeading());
        SmartDashboard.putNumber("RobotRotDeg", SwerveSubsystem.getRotation2d().getRadians());
        SmartDashboard.putBoolean("pieceInIntake", isScheduled());
        break;
    
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
