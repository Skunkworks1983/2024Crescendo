// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class MaintainHeading extends Command {

  Drivebase drivebase;
  OI oi;
  
  int fieldOrientationMultiplier;

  public MaintainHeading(Drivebase drivebase, OI oi) {
    this.drivebase = drivebase;
    this.oi = oi;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      fieldOrientationMultiplier = -1;
    } else {
      fieldOrientationMultiplier = 1;
    }

    System.out.println("Swerve Teleop Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean useHeadingControl = false;
    boolean isTargeting = drivebase.getFieldTarget().isPresent();
    boolean outsideDeadband = Math.abs(oi.getRightX()) > Constants.ROT_JOY_DEADBAND;

    // If the targeting button is being pressed, than override all other heading
    // controls and use
    // targeting.
    if (isTargeting) {
      Translation2d targetPoint = drivebase.getFieldTarget().get();

      // Uses odometry position and the specified targeting point to calculate desired
      // heading.
      headingControllerSetpoint =
          Units.radiansToDegrees(Math.atan2((targetPoint.getY() - drivebase.getRobotPose().getY()),
              (targetPoint.getX() - drivebase.getRobotPose().getX())));
      currentHeading = drivebase.getGyroAngle();
      lastSeconds = timer.getFPGATimestamp();
      useHeadingControl = true;
      hasUpdated = false;

      // If the joystick is outside of the deadband, run regular swerve.
    } else if (outsideDeadband) {

      currentHeading = drivebase.getGyroAngle();
      lastSeconds = timer.getFPGATimestamp();
      hasUpdated = false;

      // Otherwise, use the heading controller to maintain heading.
    } else {

      // Waits a second to allow extra turn momentum to dissipate.
      if (timer.getFPGATimestamp() - lastSeconds > Constants.TIME_UNTIL_HEADING_CONTROL
          && !hasUpdated) {
        currentHeading = drivebase.getGyroAngle();
        headingControllerSetpoint = currentHeading;

        // Ensures that the setpoint is only set once when maintaining heading.
        hasUpdated = true;
      }

      // If it has been enough time, useHeadingControl will be set to true.
      useHeadingControl = hasUpdated;
    }

    // If not using the heading controller, run regular swerve without heading
    // control.
    if (!useHeadingControl) {
      drivebase.setDrive(
          MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND)
              * Constants.OI_TURN_SPEED_RATIO,
          true);

      // Otherwise, set the heading controller to the desired setpoint.
    } else {
      drivebase.setHeadingController(headingControllerSetpoint);
      drivebase.setDriveTurnPos(
          MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier,
          true);
    }
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
