// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.subsystems.Drivebase;

public class TargetPosForAuto extends Command {
  /** Creates a new TargetPosForAuto. */

  Drivebase drivebase;
  FieldTarget fieldTarget;

  public TargetPosForAuto(FieldTarget target) {
    fieldTarget = target;
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setFieldTarget(fieldTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d targetPoint = drivebase.getFieldTarget().get();

    double headingControllerSetpoint =
          Units.radiansToDegrees(Math.atan2((targetPoint.getY() - drivebase.getRobotPose().getY()),
              (targetPoint.getX() - drivebase.getRobotPose().getX())));
  
    drivebase.setHeadingController(headingControllerSetpoint);
    drivebase.setDriveTurnPos(0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setFieldTarget(FieldTarget.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}