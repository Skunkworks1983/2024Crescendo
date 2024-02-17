// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PIDControllers.DrivePID;
import frc.robot.subsystems.Drivebase;

public class SetTargetingPos extends Command {

  Drivebase drivebase = Drivebase.getInstance();
  Constants.Targeting.TargetingPoint target;

  /** Creates a new SetTargetingPos. */
  public SetTargetingPos(Constants.Targeting.TargetingPoint target) {
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setTargetPoint(target.getTarget());
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
