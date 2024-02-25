// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterToAmp extends Command {

  private Shooter shooter;
  boolean isTurning;
  Rotation2d shooterAngle;

  public ShooterToAmp() {
    shooter = Shooter.getInstance();
    isTurning = false;
  }

  @Override
  public void initialize() {
    shooterAngle = Constants.Shooter.SHOOTER_MAX_POSITION_DEGREES;

    if (shooter.getShooterIndexerBeambreak2()) {
      shooter.setShooterAngle(shooterAngle);
      shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
      isTurning = true;
    } else {
      isTurning = false;
    }
  }

  @Override
  public void execute() {

    if (shooter.getShooterPivotRotation() >= Constants.Shooter.SHOOTER_MAX_POSITION_DEGREES
        .getDegrees() && isTurning) {
      shooter.setPivotMotorVelocity(Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED);
      shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    } else if (!isTurning && shooter.getShooterIndexerBeambreak2()) {
      shooter.setShooterAngle(shooterAngle);
      shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorVelocity(0);
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public boolean isFinished() {

    return shooter.getLimitSwitchOutput(true);
  }
}
