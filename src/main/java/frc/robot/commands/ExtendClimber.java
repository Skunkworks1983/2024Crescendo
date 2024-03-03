// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

// This is a stub command
public class ExtendClimber extends Command {
  /** Creates a new ExtendClimber. */
  public ExtendClimber() {
    addRequirements(SubsystemGroups.getInstance(Subsystems.CLIMBER_LEFT),
        SubsystemGroups.getInstance(Subsystems.CLIMBER_RIGHT));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Extend Climber Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Extend Climber Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
