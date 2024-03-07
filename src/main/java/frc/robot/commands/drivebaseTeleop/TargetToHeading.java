// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.HeadingController;

/** Maintain the specified heading of the drivebase. */
public class TargetToHeading extends BaseSwerveTeleop {

  Drivebase drivebase;
  HeadingController headingController;
  double headingToTarget;

  public TargetToHeading(double headingToTarget) {
    this.headingToTarget = headingToTarget;
    drivebase = Drivebase.getInstance();
    headingController = drivebase.getHeadingController();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    headingController.setSetpoint(headingToTarget);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = getDriveSpeeds();
    headingController.setSetpoint(headingToTarget);
    double desiredHeading = headingController.calculate(drivebase.getGyroAngle());
    drivebase.setDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, desiredHeading, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
