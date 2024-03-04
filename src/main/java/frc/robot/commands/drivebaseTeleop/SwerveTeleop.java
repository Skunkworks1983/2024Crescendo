// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class SwerveTeleop extends Command {

  Drivebase drivebase;
  OI oi;

  public SwerveTeleop() {
    drivebase = Drivebase.getInstance();
    oi = OI.getInstance();
    
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.setDrive(
          MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND)
              * Constants.OI_TURN_SPEED_RATIO,
          true);
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
