// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class ShootWhenReady extends Command {

  public Shooter shooter;

  public ShootWhenReady() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR) {
      shooter.setShooterIndexerSpeed(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShootMotorCoastMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
