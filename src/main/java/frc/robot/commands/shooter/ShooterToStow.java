// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.LimitSwitch;

public class ShooterToStow extends Command {

  private Shooter shooter;
  private Rotation2d shooterAngle;

  public ShooterToStow() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    shooterAngle = new Rotation2d(
        Constants.Shooter.SHOOTER_RESTING_POSITION.getRadians() + Units.degreesToRadians(Constants.Shooter.PIVOT_STOW_OFFSET));

    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public void execute() {

    if (shooter.getShooterPivotRotationInDegrees() <= Constants.Shooter.SHOOTER_RESTING_POSITION
        .getDegrees() + Constants.Shooter.PIVOT_STOW_OFFSET + 5) {
      shooter.setPivotMotorPercentOutput(-Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED);
      shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    } else {
      shooter.setPivotAngleAndSpeed(shooterAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public boolean isFinished() {

    return shooter.getLimitSwitchOutput(LimitSwitch.REVERSE_LIMIT_SWITCH);
  }
}
