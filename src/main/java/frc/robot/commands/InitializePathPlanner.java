// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousCommands.Handoff;
import frc.robot.commands.AutonomousCommands.ScoreSpeaker;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class InitializePathPlanner{
  /** Creates a new InitializePathPlanner. */
  private final SwerveSubsystem mSwerve;
  private IntakeSubsystem mIntake;
  private LauncherSubsystem mLauncher;
  private AimingSubsystem mAiming;
  private DefaultMechCommand mStateMachine;
  private boolean populateEventMaps;
  private boolean useStateMachine;
  public InitializePathPlanner(SwerveSubsystem swerve, IntakeSubsystem intake, LauncherSubsystem launcher, AimingSubsystem aiming) {
    mSwerve = swerve;
    mIntake = intake;
    mLauncher = launcher;
    mAiming = aiming;
    populateEventMaps = true;
    useStateMachine = false;
  }
  public InitializePathPlanner(SwerveSubsystem swerve){
    mSwerve = swerve;
    populateEventMaps = false;
    useStateMachine = false;
  }
  public InitializePathPlanner(SwerveSubsystem swerve, DefaultMechCommand mech){
    mSwerve = swerve;
    useStateMachine = true;
  }
  
  public void initializeNamedCommands(){
    NamedCommands.registerCommand("IntakeUntilNote", mIntake.getAutomousIntakeCommand());
    NamedCommands.registerCommand("ShootFromMidnote", new ScoreSpeaker(mLauncher, mAiming, mIntake,AimState.MIDNOTE));
    NamedCommands.registerCommand("ShootFromSubwoofer", new ScoreSpeaker(mLauncher, mAiming, mIntake,AimState.SUBWOOFER));
    NamedCommands.registerCommand("ShootFromWing", new ScoreSpeaker(mLauncher, mAiming, mIntake,AimState.WING));
    NamedCommands.registerCommand("ShootFromLine", new ScoreSpeaker(mLauncher, mAiming, mIntake,AimState.LINE));
    NamedCommands.registerCommand("StartLauncherSW", mLauncher.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER));
    NamedCommands.registerCommand("StartWristSW", mLauncher.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER));
    NamedCommands.registerCommand("Handoff", new Handoff(mIntake, mLauncher));
  }

  public void initializeEmptyNamedCommands(){
    NamedCommands.registerCommand("IntakeUntilNote", new SequentialCommandGroup(new PrintCommand("Intaking until note enters")));
    NamedCommands.registerCommand("ShootFromMidnote", new SequentialCommandGroup(new PrintCommand("Shoot Note from Middle Position"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromSubwoofer", new SequentialCommandGroup(new PrintCommand("Shoot Note From Subwoofer"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromWing", new SequentialCommandGroup(new PrintCommand("Shoot Note from Wing Edge"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromLine", new SequentialCommandGroup(new PrintCommand("Shoot Note from Autonomous Line"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("Handoff", new SequentialCommandGroup(new PrintCommand("Shoot Note from Wing Edge"), new WaitCommand(0.5)));
  }


  // Called when the command is initially scheduled.
  public void initialize() {
    AutoBuilder.configureHolonomic(
    SwerveSubsystem::getRobotPose, 
    mSwerve::resetRobotPose,
    SwerveSubsystem::getChassisSpeeds, 
    mSwerve::setChassisSpeed,
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                                      // your Constants class
          new PIDConstants(5, 0.0, 0), // Translation PID constants
          new PIDConstants(5, 0.0, 0), // Rotation PID constants
          3, // Max module speed, in m/s
          0.4669, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig(true, true) // Default path replanning config. See the API for the options
                              // here
              
    ), this::determineFieldOrientation, mSwerve);
    if (populateEventMaps){
      initializeNamedCommands();
    } else{
      initializeEmptyNamedCommands();
    }

  }

  public boolean determineFieldOrientation(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}
