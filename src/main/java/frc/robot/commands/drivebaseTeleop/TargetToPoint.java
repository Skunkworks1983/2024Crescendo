// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PIDControllers.HeadingControlPID;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.subsystems.Drivebase;

public class TargetToPoint extends SwerveTeleop {

  Drivebase drivebase;
  FieldTarget fieldTarget;
  double headingControllerSetpoint;

  public TargetToPoint(FieldTarget fieldTarget) {
    this.fieldTarget = fieldTarget;
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (fieldTarget.get().isPresent()) {
      Translation2d targetPoint = fieldTarget.get().get().toTranslation2d();

      // Uses odometry position and the specified targeting point to calculate desired
      // heading.
      headingControllerSetpoint = Units
          .radiansToDegrees(Math.atan2((targetPoint.getY() - drivebase.getRobotPose().getY()),
              (targetPoint.getX() - drivebase.getRobotPose().getX())));
      
      ChassisSpeeds speeds = getChassisSpeedsUsingJoysticks();

      drivebase.setHeadingControllerSetpoint(headingControllerSetpoint);
      drivebase.setDriveWithHeadingControl(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
