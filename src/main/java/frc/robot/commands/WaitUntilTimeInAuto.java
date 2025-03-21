// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilTimeInAuto extends Command {
  /** Creates a new WaitUntilTimeInAuto. */
  int waitUntil;

  public WaitUntilTimeInAuto(int timeBeforeEndOfAuto) {
    waitUntil = timeBeforeEndOfAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Wait Until Time In Auto Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Wait Until Time In Auto Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() <= waitUntil;
  }
}
