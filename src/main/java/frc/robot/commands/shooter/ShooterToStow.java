// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterToStow extends Command {

  private Shooter shooter;

  public ShooterToStow() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    Rotation2d shooterAngle = Constants.Shooter.SHOOTER_RESTING_POSITION;

    shooter.setShooterAngle(shooterAngle);
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public void execute() {

    if (shooter.getShooterPivotRotationInDegrees() >= Constants.Shooter.SHOOTER_RESTING_POSITION
        .getDegrees()) {
      shooter.setPivotMotorPercentOutput(-Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED);
      shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterAngle(
        new Rotation2d(Units.degreesToRadians(shooter.getShooterPivotRotationInDegrees())));
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public boolean isFinished() {

    return shooter.getLimitSwitchOutput(false);
  }
}
