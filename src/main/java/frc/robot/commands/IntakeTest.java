// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Intake;

public class IntakeTest extends Command {
  Intake m_intake;
  boolean isFinished;
  /** Creates a new IntakeTest. */
  public IntakeTest(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testGetID();
    //put the rest of the tests here in init
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Input.getleftBumperXbox()) {
      m_intake.runMotorRaw(-1.0);
      SequentialCommandGroup wait = new SequentialCommandGroup(new WaitCommand(1));
      wait.schedule();
      m_intake.stopMotor();
      wait.schedule();
      m_intake.runMotorRaw(1.0);
    }
  }

  public void testGetID() {
    Intake intake = new Intake(4, 2);
    if (intake.getID() == 4) {
      System.out.println("Test Worked for ID");
    } else {
      System.out.println("Test Failed for GetID");
    }
  }

  public void checkGamePieceInIntake() {
    SmartDashboard.putBoolean("Game Piece In Intake", m_intake.gamePieceInIntake());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
