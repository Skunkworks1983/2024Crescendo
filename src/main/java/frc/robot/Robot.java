// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command swerve;
  private RobotContainer m_robotContainer;

  Drivebase drivebase = Drivebase.getInstance();
  OI oi = OI.getInstance();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
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

    if (swerve != null) {
      swerve.cancel();
    }

    //m_autonomousCommand = Drivebase.getInstance().followPathCommand("testPath");
    m_autonomousCommand = drivebase.followAutoTrajectory("Copy of New Auto");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      
    }
        System.out.println("autonomousInit:" + Drivebase.getInstance().getRobotPose().getX() + ", " +Drivebase.getInstance().getRobotPose().getY());

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    System.out.println("autonomousExit:" + Drivebase.getInstance().getRobotPose().getX() + ", " +Drivebase.getInstance().getRobotPose().getY());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    swerve=new SwerveTeleop(drivebase, oi);
    swerve.schedule();
    //drivebase.setSwerveAsDefaultCommand();
    //directionTest.schedule();
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
