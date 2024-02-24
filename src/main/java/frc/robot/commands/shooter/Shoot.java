// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

  private Shooter shooter;

  public Shoot() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT);
    System.out.println("Shoot Command Initialize");
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerMotorCoastMode();
    System.out.println("Shoot Command End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
