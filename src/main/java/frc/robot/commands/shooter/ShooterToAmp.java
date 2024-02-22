// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.PivotPosition;

public class ShooterToAmp extends Command {

  public Shooter shooter;
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
      shooter.setShooterAngle(shooterAngle, PivotPosition.SHOOTER_TO_AMP);
      isTurning = true;
    } else {
      isTurning = false;
    }
  }

  @Override
  public void execute() {

    if (shooter.getShooterPivotRotation() >= Constants.Shooter.SHOOTER_MAX_POSITION_DEGREES
        .getDegrees() && isTurning) {
      shooter.setShooterAngleVelocity(Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED,
          PivotPosition.SHOOTER_TO_AMP);
    } else if (!isTurning && shooter.getShooterIndexerBeambreak2()) {
      shooter.setShooterAngle(shooterAngle, PivotPosition.SHOOTER_TO_AMP);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // at the end of this command, we will always be going to stow unless another command like
    // target to speaker is running. Oi wull be controling this, so this command does exactly what
    // it says
  }

  @Override
  public boolean isFinished() {

    return shooter.getLimitSwitchOutput(true);
  }
}
