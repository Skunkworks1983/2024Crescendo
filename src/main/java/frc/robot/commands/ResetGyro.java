// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class ResetGyro extends Command {
  /** Creates a new resetGyro. */
  Drivebase drivebase;

  public ResetGyro() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Reset Gyro Command Initialize");
    drivebase.resetGyroOffset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Reset Gyro Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
