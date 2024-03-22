// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tuningCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;
import frc.robot.utils.ShooterAimUtils;

public class InterpolationAimShooterCommand extends Command {
  private Shooter shooter;
  Rotation2d shooterAngle;
  double driveToleranceTicks;
  Timer timer;
  Drivebase drivebase;
  boolean isMoving = true;

  public InterpolationAimShooterCommand() {
    shooter = Shooter.getInstance();
    drivebase = Drivebase.getInstance();

    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSetpoint(Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED);
    resetAim();
    System.out.println("interpolation aim shooter command init");
  }

  public void resetAim() {
    Pose2d pose = drivebase.getRobotPose();
    shooterAngle = Rotation2d.fromDegrees(ShooterAimUtils.calculateInterpolatedAimAngle(
        ShooterAimUtils.calculateInputForInterpolatedAimAngle(pose.getTranslation())));

    // ensures aim does not result in angles that break the shooter.
    if (shooterAngle.getDegrees() > 90.0) {
      shooterAngle = Rotation2d.fromDegrees(90);
    } else if (shooterAngle.getDegrees() < Constants.Shooter.SHOOTER_RESTING_POSITION
        .getDegrees()) {
      shooterAngle = Constants.Shooter.SHOOTER_RESTING_POSITION;
    }
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);

    double speed = new Translation2d(drivebase.getFieldRelativeSpeeds().vxMetersPerSecond,
        drivebase.getFieldRelativeSpeeds().vyMetersPerSecond).getNorm();
    if (speed > Constants.ShooterInterpolationConstants.MINIMUM_SPEED_TO_RE_AIM) {
      driveToleranceTicks++;
    } else {
      driveToleranceTicks = 0;
    }

    // Resets aim if drivebase is moving for long enough, indicating that aim value is too old
    if (driveToleranceTicks > Constants.ShooterInterpolationConstants.NUMBER_OF_TICKS_GOING_TO_FAST_TO_RE_AIM) {
      resetAim();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("interpolation aim shooter command end");
    shooter.setPivotMotorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}