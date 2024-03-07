// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivebase;

/** Regular swerve with heading controlled by joysticks. */
public class RegularSwerve extends BaseSwerveTeleop {

  Drivebase drivebase;

  public RegularSwerve() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds speeds = getDriveSpeeds();
    drivebase.setDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
