// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DIRECTION;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class DirectionTest extends Command {
  
  /** Creates a new SwerveTest. */
  
  Drivebase drivebase;
  OI oi;
  DIRECTION direction;
  boolean fieldRelative;

  public DirectionTest(Drivebase drivebase, OI oi, DIRECTION direction, boolean fieldRelative) {
    this.direction = direction;
    this.fieldRelative = fieldRelative;
    this.drivebase = drivebase;
    this.oi = oi;

    addRequirements(drivebase, oi);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(direction) {
      case X:
        drivebase.setDrive(oi.getLeftY(), 0, 0, fieldRelative);
        break;
      case Y:
        drivebase.setDrive(0, oi.getLeftY(), 0, fieldRelative);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
