// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DriveFastTest extends Command {//8.25 feet per second
  /** Creates a new DriveFastTest. */
  Drivebase drivebase;
  public DriveFastTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivebase = Drivebase.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivebase.setDrive(
      0.0,
      8.25,
      0.0,
      false);
drivebase.setBreakMode(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivebase.setDrive(0,
    0,
    0,
    false);

drivebase.setBreakMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
