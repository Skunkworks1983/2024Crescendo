// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.untested;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterToSource extends Command {

  private Shooter shooter;
  boolean isTurning;
  Rotation2d shooterAngle;

  public ShooterToSource() {
    shooter = Shooter.getInstance();
    isTurning = false;
    shooterAngle = new Rotation2d(Units.degreesToRadians(40));
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    System.out.println(
        "Shooter to Source Command Initialize");
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
    System.out.println(
        "Shooter to Source Command End");
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
