// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.HeadingController;

/** Maintain a heading with the drivebase. */
public class MaintainHeading extends BaseSwerveTeleop {

  Drivebase drivebase;
  HeadingController headingController;
  double headingToMaintain;
  boolean setpointIsSet;


  public MaintainHeading() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    setpointIsSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!setpointIsSet && headingController.useHeadingControl()) {
      headingToMaintain = drivebase.getGyroAngle();
      headingController.setSetpoint(headingToMaintain);
    }

    double degreesPerSecond = headingController.calculate(drivebase.getGyroAngle());
    ChassisSpeeds speeds = getDriveSpeeds();
    drivebase.setDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, degreesPerSecond, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Swerve Teleop Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
