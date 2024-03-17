// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CollectNote;
import frc.robot.commands.CollectorStow;
import frc.robot.commands.IntakeNoteToIndexerAuto;
import frc.robot.commands.LowerCollector;
import frc.robot.commands.LowerCollectorAndIntakeToIndexer;
import frc.robot.commands.NoteFloorToShooter;
import frc.robot.commands.WaitDuration;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.LoadPieceShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootWhenReady;
import frc.robot.commands.shooter.ShooterToAmp;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector.LimitSwitch;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command swerve;
  private SendableChooser<Command> autoChooser;

  OI oi;
  Drivebase drivebase;
  Shooter shooter;
  Indexer indexer;
  Collector collector;

  @Override
  public void robotInit() {
    oi = OI.getInstance();
    drivebase = Drivebase.getInstance();
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    collector = Collector.getInstance();
    //enable the driver camera
    CameraServer.startAutomaticCapture();

  //wait times
  NamedCommands.registerCommand("WaitOneSecond", new WaitDuration(1.0));
  NamedCommands.registerCommand("WaitHalfSecond", new WaitDuration(0.5));

  // Collector 
  NamedCommands.registerCommand("LowerCollector", new LowerCollector());
  NamedCommands.registerCommand("CollectorStow", new CollectorStow());
  NamedCommands.registerCommand("CollectNote", new CollectNote());

  // Shooter
  NamedCommands.registerCommand("ShootNote",new Shoot());
  NamedCommands.registerCommand("SpinUpFlywheel", new FlywheelSpinup());
  NamedCommands.registerCommand("ShooterToAmp", new ShooterToAmp());
  NamedCommands.registerCommand("ShootWhenReady", new ShootWhenReady());
  NamedCommands.registerCommand("LoadPieceShooter", new LoadPieceShooter());

  //indexer
  NamedCommands.registerCommand("LowerCollectorAndInatake", new LowerCollectorAndIntakeToIndexer());
  NamedCommands.registerCommand("IntakeNoteToIndexer", new IntakeNoteToIndexerAuto());

  NamedCommands.registerCommand("NoteFloorToShooter", new NoteFloorToShooter());


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Indexer Beambreak", indexer.getBeamBreakSensor());
    SmartDashboard.putBoolean("Shooter Beambreak One", shooter.getShooterIndexerBeambreak1());
    SmartDashboard.putBoolean("Shooter Beambreak Two", shooter.getShooterIndexerBeambreak2());
    SmartDashboard.putNumber("Collector Angle", collector.getCollectorPos());
  }

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("Indexer Beambreak", indexer.getBeamBreakSensor());
    SmartDashboard.putBoolean("Shooter Beambreak One", shooter.getShooterIndexerBeambreak1());
    SmartDashboard.putBoolean("Shooter Beambreak Two", shooter.getShooterIndexerBeambreak2());
    SmartDashboard.putBoolean("Collector Limit Forward", collector.getLimitSwitchOutput(LimitSwitch.FORWARD_LIMIT_SWITCH));
    SmartDashboard.putBoolean("Collector Limit Backward", collector.getLimitSwitchOutput(LimitSwitch.REVERSE_LIMIT_SWITCH));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // cancels swerve teleop command to make sure it does not interfere with auto
    if (swerve != null) {
      swerve.cancel();
    }

    Command currentAutonomousCommand = autoChooser.getSelected();
    if (currentAutonomousCommand != null) {
      currentAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    drivebase.setSwerveAsDefaultCommand();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
