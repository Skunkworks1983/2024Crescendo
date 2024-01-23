// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;


public class SwerveTeleop extends Command {
  
  Drivebase drivebase;
  OI oi; 
  boolean inJoystickDeadzone = true;

  public SwerveTeleop(Drivebase drivebase, OI oi) {
    this.drivebase = drivebase;
    this.oi = oi;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(oi.getRightX())<=Constants.X_JOY_DEADBAND && !inJoystickDeadzone){
      drivebase.setHeadingController(-drivebase.getGyroAngle());
      inJoystickDeadzone = true;
    }
    if(Math.abs(oi.getRightX())>Constants.X_JOY_DEADBAND && inJoystickDeadzone){
      inJoystickDeadzone = false;
    }

    if(!inJoystickDeadzone){

      drivebase.setDrive(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND) * Constants.OI_TURN_SPEED_RATIO,
        true
      );
    }
    else{
      drivebase.setDriveDeadband(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        true
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}