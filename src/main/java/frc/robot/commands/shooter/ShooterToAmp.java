// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.LimitSwitch;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterToAmp extends Command {

  private Shooter shooter;
  Rotation2d shooterAngle;

  public ShooterToAmp() {
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    shooterAngle = Constants.Shooter.SHOOTER_MAX_POSITION;

    shooter.setPivotAngleAndSpeed(shooterAngle);
    shooter.setFlywheelSetpoint(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    System.out.println("Shooter to Amp Command Initialize");
  }

  @Override
  public void execute() {
    if (shooter.getShooterPivotRotationInDegrees() >= Constants.Shooter.SHOOTER_MAX_POSITION
        .getDegrees()) {
      shooter.setPivotMotorPercentOutput(Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED);
      shooter.setFlywheelSetpoint(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    } else {
      shooter.setPivotAngleAndSpeed(shooterAngle);
      shooter.setFlywheelSetpoint(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
    shooter.setFlywheelSetpoint(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    System.out.println("Shooter to Amp Command End");
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
