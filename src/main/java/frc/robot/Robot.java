// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.CollationElementIterator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.WaitDuration;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.LimitSwitch;

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
    NamedCommands.registerCommand("WaitOneSecond", new WaitDuration(1.0));
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
    SmartDashboard.putNumber("Collector Angle", collector.getCollectorPos());
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
