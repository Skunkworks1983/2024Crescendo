// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.WaitDuration;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command swerve;
  private RobotContainer m_robotContainer;
  private SendableChooser<Command> autoChooser;

  Drivebase drivebase = Drivebase.getInstance();
  OI oi = OI.getInstance();

  @Override
  public void robotInit() {
      m_robotContainer = new RobotContainer();
      NamedCommands.registerCommand("WaitOneSecond", new WaitDuration(1.0));
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //cancels swerve teleop command to make sure it does not interfere with auto
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
    swerve=new SwerveTeleop(drivebase, oi);
    swerve.schedule();
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
