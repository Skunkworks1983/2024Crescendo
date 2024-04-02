// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class TurnToHeading extends Command {

  Drivebase drivebase;
  double headingSetpoint;
  
  /** Command to turn to a heading during auto */
  public TurnToHeading(double headingSetpoint) {
    this.headingSetpoint = headingSetpoint;
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    drivebase.setHeadingController(headingSetpoint);
  }

  @Override
  public void execute() {
    drivebase.setDriveTurnPos(0.0, 0.0, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return drivebase.headingControllerAtSetpoint();
  }
}
