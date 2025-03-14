// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class autoAmp extends Command {

  Drivebase drivebase;
  FieldTarget fieldTarget;
  double xTargetPos;
  int fieldOrientationMultiplier;
  OI oi;
  double aligningKP = 0.7;
  double maxSpeedAlignCap = 0.75;
  double xJoystickSpeedReduction = 1.0/3.0;

  public autoAmp(OI oi) {
    fieldTarget = FieldTarget.AMP;
    this.oi = oi;
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xTargetPos = Constants.Targeting.FieldTarget.AMP.get().get().getX();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      fieldOrientationMultiplier = -1;
      xTargetPos = Constants.FIELD_X_LENGTH / 2 + (Constants.FIELD_X_LENGTH / 2 - xTargetPos);
      System.out.println("Red");
    } else {
      fieldOrientationMultiplier = 1;
      System.out.println("Blue");
    }

    drivebase.setFieldTarget(fieldTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d targetPoint = drivebase.getFieldTarget().get();

    // Uses odometry position and the specified targeting point to calculate desired
    // heading.
    drivebase.setHeadingController(
        Units.radiansToDegrees(Math.atan2((targetPoint.getY() - drivebase.getRobotPose().getY()),
            (targetPoint.getX() - drivebase.getRobotPose().getX()))));

    double distanceFromAmp = xTargetPos - drivebase.getRobotPose().getX();
    drivebase.setDriveTurnPos(
        Math.max(Math.min((Math.max(Math.min(distanceFromAmp * aligningKP, maxSpeedAlignCap), -maxSpeedAlignCap)
            + (MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * xJoystickSpeedReduction)
                * fieldOrientationMultiplier),
            1), -1) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND)
            * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setFieldTarget(FieldTarget.NONE);
  }

  // Returns true when the command should end.%
  @Override
  public boolean isFinished() {
    return false;
  }
}
