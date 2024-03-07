// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.commands.InitializePathPlanner;
import frc.robot.commands.SwerveFrictionCharacterization;
import frc.robot.commands.SwerveVoltageCharacterization;
import frc.robot.commands.AutonomousCommands.LaunchFromHandoff;
import frc.robot.commands.AutonomousCommands.ScoreSpeaker;
import frc.robot.constants.AimState;
import frc.robot.subsystems.Autoaim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private SendableChooser<Command> pathSelector = new SendableChooser<>();
  private RobotContainer robotContainer = new RobotContainer();
  private DefaultMechCommand mDefaultMechCommand = new DefaultMechCommand(robotContainer.getIntakeSubsystem(), robotContainer.getLauncher(), robotContainer.getAimingSubsystem());
;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
   
    new InitializePathPlanner(robotContainer.getSwerveSubsystem(),robotContainer.getIntakeSubsystem(),robotContainer.getLauncher(),robotContainer.getAimingSubsystem()).initialize();
    
    SmartDashboard.putData("Pick your Auton...",pathSelector);

    // pathSelector.addOption("kSCharacterization", new SwerveFrictionCharacterization(robotContainer.getSwerveSubsystem()));
    // pathSelector.addOption("kVCharacterization", new SwerveVoltageCharacterization(robotContainer.getSwerveSubsystem()));
    // pathSelector.addOption("4 Piece LC SW", AutoBuilder.buildAuto("4_Piece_SW_Accurate"));
    // pathSelector.addOption("4 Piece LC Line", AutoBuilder.buildAuto("4_Piece_Accurate"));
    // pathSelector.addOption("4 Piece LC Hail Mary", AutoBuilder.buildAuto("4_Piece_Bum"));
    // pathSelector.addOption("4 Piece LC SW Hail Mary", AutoBuilder.buildAuto("4_Piece_SW_Bum"));
    // pathSelector.addOption("4 Piece Subwoofer", AutoBuilder.buildAuto("4_Piece_V1"));
    // pathSelector.addOption("4 Piece Midnote", AutoBuilder.buildAuto("4_Piece_V2"));
    // pathSelector.addOption("5 Meter Test", AutoBuilder.buildAuto("5_Meter_Test"));
    // pathSelector.addOption("None", new PrintCommand("Damn that sucks"));


    // pathSelector.addOption("4 Piece SW Method 2", 
    // new SequentialCommandGroup(
    //   new LaunchFromHandoff(robotContainer.getAimingSubsystem(), robotContainer.getLauncher(), AimState.SUBWOOFER),
    //   AutoBuilder.followPath(PathPlannerPath.fromPathFile("2nd_Piece_SW")),
    //   new LaunchFromHandoff(robotContainer.getAimingSubsystem(), robotContainer.getLauncher(), AimState.SUBWOOFER),
    //   AutoBuilder.followPath(PathPlannerPath.fromPathFile("3rd_Piece_SW")),
    //   new LaunchFromHandoff(robotContainer.getAimingSubsystem(), robotContainer.getLauncher(), AimState.SUBWOOFER),
    //   AutoBuilder.followPath(PathPlannerPath.fromPathFile("4th_Piece_SW")),
    //   new LaunchFromHandoff(robotContainer.getAimingSubsystem(), robotContainer.getLauncher(), AimState.SUBWOOFER)
    // ));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putBoolean("IsFinishedScoreSpeaker", ScoreSpeaker.mCommand.isFinished());
    SmartDashboard.putBoolean("IsScheduledScoreSpeaker", ScoreSpeaker.mCommand.isScheduled());
    SmartDashboard.updateValues();


    CommandScheduler.getInstance().run();

    Autoaim.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Not used
  }

  @Override
  public void disabledPeriodic() {
    //Not used
  }

  @Override
  public void teleopInit() {
    //Not used
  }

  @Override
  public void teleopPeriodic() {
    //Not used
    mDefaultMechCommand.execute();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Command autonomousCommand = pathSelector.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    CANSparkLowLevel.enableExternalUSBControl(true);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CANSparkLowLevel.enableExternalUSBControl(true);
  }
}
