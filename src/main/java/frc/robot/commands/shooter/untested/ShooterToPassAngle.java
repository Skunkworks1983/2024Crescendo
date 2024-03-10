// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.untested;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterToPassAngle extends Command {

  private Shooter shooter;
  Rotation2d shooterAngle;
  boolean isTurning;

  public ShooterToPassAngle() {
    shooter = Shooter.getInstance();
    shooterAngle = new Rotation2d(Units.degreesToRadians(Constants.Shooter.PASS_ANGLE_DEGREES));
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSetpoint(Constants.Shooter.PASS_FLYWHEEL_SPEED);
    if(shooter.getShooterIndexerBeambreak1() || shooter.getShooterIndexerBeambreak2()) {
      isTurning = true;
      shooter.setPivotAngleAndSpeed(shooterAngle);
    }
    else {
      isTurning = false;
    }
    System.out.println(
        "Shooter to Pass Anlge Command Initialize");
  }

  @Override
  public void execute() {
    if (!isTurning && shooter.getShooterIndexerBeambreak1() || shooter.getShooterIndexerBeambreak2()) {
      shooter.setPivotAngleAndSpeed(shooterAngle);
      isTurning = true;
    } else if (isTurning) {
      shooter.setPivotAngleAndSpeed(shooterAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
    System.out.println(
        "Shooter to Pass Angle Command End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
