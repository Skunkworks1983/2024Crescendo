// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.PivotPosition;

public class ShooterToStow extends Command {

  public Shooter shooter;

  public ShooterToStow() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    Rotation2d shooterAngle = Constants.Shooter.SHOOTER_RESTING_POSITION_DEGREES;

    shooter.setShooterAngle(shooterAngle, PivotPosition.SHOOTER_TO_STOW);
  }

  @Override
  public void execute() {

    if (shooter.getShooterPivotRotation() >= Constants.Shooter.SHOOTER_RESTING_POSITION_DEGREES.getDegrees()) {
      shooter.setShooterAngleVelocity(-Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED, PivotPosition.SHOOTER_TO_STOW);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    return shooter.getLimitSwitchOutput(false);
  }
}
