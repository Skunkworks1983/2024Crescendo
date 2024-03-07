// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import frc.robot.subsystems.Drivebase;

// TODO: Finish writing this command with new piece detection code
/** Center the drivebase on a game piece. */
public class CenterOnPiece extends BaseSwerveTeleop {

  Drivebase drivebase;

  public CenterOnPiece() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
