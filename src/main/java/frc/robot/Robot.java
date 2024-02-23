// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CollectNotes;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.WaitDuration;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command swerve;
  private SendableChooser<String> autoPositionChooser;
  private SendableChooser<String> autoDetailChooser;

  OI oi;
  Drivebase drivebase;

  @Override
  public void robotInit() {
    oi = OI.getInstance();
    drivebase = Drivebase.getInstance();
    NamedCommands.registerCommand("WaitOneSecond", new WaitDuration(1.0));
    NamedCommands.registerCommand("WaitHalfSecond", new WaitDuration(0.5));
    //NamedCommands.registerCommand("CollectNotes", new CollectNotes());
    buildAutoChooser("");
    SmartDashboard.putData("Auto Position Chooser", autoPositionChooser);
    SmartDashboard.putData("Auto Chooser", autoDetailChooser);
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
    // cancels swerve teleop command to make sure it does not interfere with auto
    if (swerve != null) {
      swerve.cancel();
    }

    Command currentAutonomousCommand = null;//autoDetailChooser.getSelected();
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

   /**
   * Create and populate a sendable chooser with all PathPlannerAutos in the project
   *
   * @param defaultAutoName The name of the auto that should be the default option. If this is an
   *     empty string, or if an auto with the given name does not exist, the default option will be
   *     Commands.none()
   * @return SendableChooser populated with all autos
   */
  public void buildAutoChooser(String defaultAutoName) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<String> posChooser = new SendableChooser<>();
    SendableChooser<Command> chooser = new SendableChooser<>();
    SendableChooser<String> detailChooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    List<String> positions = new ArrayList<String>();
    List<String> details = new ArrayList<String>();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      int dotLocation = (autoName.indexOf("."));
      if (dotLocation > 0){
        String position = autoName.substring(0,autoName.indexOf("."));
        String detail = autoName.substring(dotLocation + 1);
        if (!positions.contains(position)) { 
          positions.add(position);  
        }
        if (!details.contains(detail)) { 
          details.add(detail);  
        }
      }

      PathPlannerAuto auto = new PathPlannerAuto(autoName);

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

   /* if (defaultOption == null) {
      chooser.setDefaultOption("None", Commands.none());
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
    }*/

    options.forEach(auto -> chooser.addOption(auto.getName(), auto));
    positions.forEach(position -> posChooser.addOption(position, position));
    details.forEach(detail -> detailChooser.addOption(detail, detail));

    autoDetailChooser = detailChooser;
    autoPositionChooser = posChooser;
  }
}
