// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class SwerveTeleop extends Command {

  Drivebase drivebase;
  OI oi;

  // this is updated to the robots current angle when using the targeting button or not in the turn
  // joystick deadzone. Used for heading correction when not using the targeting button and in the
  // turn joystick deadzone
  double setpointHeadingControl = 0.0;

  // parts of the code set this variable, and then the variable is used to tell the drive command
  // that turns to a certan angle where to turn to
  double desiredHeadingSetpoint = 0.0;

  // this var will be swaped to -1 if we are on the red Alliance
  int isFlipped = 1;

  Timer timer;
  double timeAtLastInput;

  // has updated ensures that desired heading is only set once, driver stops rotating.
  // if it is false and robot should maintain current heading, desiredHeadingSetpoint will set to
  // current heading.
  // Once it is set to true, robot will rotate to desiredHeadingSetpoint.
  boolean hasUpdated = false;

  public SwerveTeleop(Drivebase drivebase, OI oi) {
    timer = new Timer();
    timeAtLastInput = timer.getFPGATimestamp();
    this.drivebase = drivebase;
    this.oi = oi;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      isFlipped = -1;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angularVelocity = 0;
    boolean useHeadingControl = false;

    if (Math.abs(oi.getRightX()) > Constants.ROT_JOY_DEADBAND) {
      angularVelocity = oi.getRightX() * Constants.OI_TURN_SPEED_RATIO;
      setpointHeadingControl = drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
      hasUpdated = false;
    } else if (oi.getTargetingButton()) {
      // calculates our current position on the field and where we are targeting to and figures out
      // the angle to point at
      desiredHeadingSetpoint = Units.radiansToDegrees(
          Math.atan2((Constants.TARGETING_POSITION_Y - drivebase.getRobotPose().getY()),
              (Constants.TARGETING_POSITION_X - drivebase.getRobotPose().getX())));
      setpointHeadingControl = drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
      hasUpdated = false;
      useHeadingControl = true;
    } else {
      // waits a second to allow for extra turn momentum to dissipate
      if (timer.getFPGATimestamp() - timeAtLastInput > Constants.TIME_UNTIL_HEADING_CONTROL
          && !hasUpdated) {
        setpointHeadingControl = drivebase.getGyroAngle();
        desiredHeadingSetpoint = setpointHeadingControl;

        // ensures setpoint is only set once when maintaining heading.
        hasUpdated = true;
      }
      useHeadingControl = hasUpdated;

    }

    if (!useHeadingControl) {
      drivebase.setDrive(
          MathUtil.applyDeadband(oi.getLeftY() * isFlipped, Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          MathUtil.applyDeadband(oi.getLeftX() * isFlipped, Constants.Y_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          angularVelocity, true);
    } else {
      drivebase.setHeadingController(desiredHeadingSetpoint);
      drivebase.setDriveTurnPos(
          MathUtil.applyDeadband(oi.getLeftY() * isFlipped, Constants.X_JOY_DEADBAND)
              * Constants.OI_DRIVE_SPEED_RATIO,
          MathUtil.applyDeadband(oi.getLeftX() * isFlipped, Constants.Y_JOY_DEADBAND)
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
