// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class SwerveTeleop extends Command {

  Drivebase drivebase;
  OI oi;

  // Used as the setpoint for heading correction when joystick and targeting button are not being
  // used.
  double currentHeading = 0.0;

  // Tells the heading contoller what heading to turn to.
  double headingControllerSetpoint = 0.0;

  private final Timer timer = new Timer();
  double lastSeconds;

  // Ensures the the heading contoller is only set once.
  boolean hasUpdated = false;

  public SwerveTeleop(Drivebase drivebase, OI oi) {
    this.drivebase = drivebase;
    this.oi = oi;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    lastSeconds = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean useHeadingControl = false;
    boolean isTargeting = drivebase.getTargetPoint().isPresent();
    boolean outsideDeadband = Math.abs(oi.getRightX()) > Constants.ROT_JOY_DEADBAND;
    double seconds = timer.get();

    // If the targeting button is being pressed, than override all other heading controls and use
    // targeting.
    if (isTargeting) {
      Translation2d targetPoint = drivebase.getTargetPoint().get();

      // Uses odometry position and the specified targeting point to calculate desired heading.
      headingControllerSetpoint =
          Units.radiansToDegrees(Math.atan2((targetPoint.getY() - drivebase.getRobotPose().getY()),
              (targetPoint.getX() - drivebase.getRobotPose().getX())));
      currentHeading = drivebase.getGyroAngle();
      lastSeconds = seconds;
      useHeadingControl = true;
      hasUpdated = false;

    // If the joystick is outside of the deadband, run regular swerve.
    } else if (outsideDeadband) {

      currentHeading = drivebase.getGyroAngle();
      lastSeconds = seconds;
      hasUpdated = false;

    // Otherwise, use the heading controller to maintain heading.
    } else {

      // Waits a second to allow extra turn momentum to dissipate.
      if (seconds - lastSeconds > Constants.TIME_UNTIL_HEADING_CONTROL && !hasUpdated) {
        currentHeading = drivebase.getGyroAngle();
        headingControllerSetpoint = currentHeading;

        // Ensures that the setpoint is only set once when maintaining heading.
        hasUpdated = true;
      }

      // If it has been enough time, useHeadingControl will be set to true.
      useHeadingControl = hasUpdated;
    }


    // If not using the heading controller, run regular swerve without heading control.
    if (!useHeadingControl) {
      drivebase.setDrive(
          MathUtil.applyDeadband(oi.getLeftX(), Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          MathUtil.applyDeadband(oi.getLeftY(), Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND)
              * Constants.OI_TURN_SPEED_RATIO,
          true);

      // Otherwise, set the heading controller to the desired setpoint.
    } else {
      drivebase.setHeadingController(headingControllerSetpoint);
      drivebase.setDriveTurnPos(
          MathUtil.applyDeadband(oi.getLeftX(), Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          MathUtil.applyDeadband(oi.getLeftY(), Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          true);
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
