// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterToAngle extends Command {

  private Shooter shooter;
  boolean isTurning;
  Rotation2d shooterAngle;

  public ShooterToAngle(double angleDegrees) {
    shooter = Shooter.getInstance();
    isTurning = false;
    shooterAngle = new Rotation2d(Units.degreesToRadians(angleDegrees));
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
    shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
