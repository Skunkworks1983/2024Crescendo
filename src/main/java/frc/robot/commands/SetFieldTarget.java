// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.constants.Constants.Targeting.FieldTarget;

public class SetFieldTarget extends Command {

  Drivebase drivebase;
  FieldTarget fieldTarget;

  /** Creates a new SetTargetingPoint. */
  public SetFieldTarget(FieldTarget fieldTarget) {
    this.fieldTarget = fieldTarget;
    drivebase = Drivebase.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setFieldTarget(fieldTarget);
    System.out.println("Set Field Target Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setFieldTarget(FieldTarget.NONE);
    System.out.println("Set Field Target Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
