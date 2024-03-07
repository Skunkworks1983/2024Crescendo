// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

/** Basic swerve drive teleop functionalities. */
public class BaseSwerveTeleop extends Command {

  Drivebase drivebase;
  OI oi;
  int fieldOrientationMultiplier;

  public BaseSwerveTeleop() {
    drivebase = Drivebase.getInstance();
    oi = OI.getInstance();
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      fieldOrientationMultiplier = -1;
    } else {
      fieldOrientationMultiplier = 1;
    }
  }

  /** Get the speeds of the drivebase using the joysticks. */
  public ChassisSpeeds getDriveSpeeds() {
    return new ChassisSpeeds(
        Units.feetToMeters(MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND)
            * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier),
        Units.feetToMeters(MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND)
            * Constants.OI_DRIVE_SPEED_RATIO * fieldOrientationMultiplier),
        Units.feetToMeters(MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND)
            * Constants.OI_TURN_SPEED_RATIO));
  }
}
