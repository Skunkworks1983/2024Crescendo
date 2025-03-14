// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class SetRobotRelativeSwerve extends Command {
  /** Creates a new SetRobotRelitiveSwerve. */
  Drivebase drivebase;

  public SetRobotRelativeSwerve() {
    drivebase = Drivebase.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setRobotRelative();
    System.out.println("Set Robot Relitive swerve init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setFieldRelative();
    System.out.println("Set Robot Relitive swerve end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
