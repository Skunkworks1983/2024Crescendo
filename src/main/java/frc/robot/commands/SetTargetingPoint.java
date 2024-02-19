// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class SetTargetingPoint extends Command {

  Drivebase drivebase = Drivebase.getInstance();
  OI oi = OI.getInstance();
  Constants.Targeting.TargetingPoint targetPoint;

  /** Creates a new SetTargetingPos. */
  public SetTargetingPoint(Constants.Targeting.TargetingPoint targetPoint) {
    this.targetPoint = targetPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setTargetPoint(targetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setTargetPoint(Constants.Targeting.TargetingPoint.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
