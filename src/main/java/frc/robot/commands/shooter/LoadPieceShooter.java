// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class LoadPieceShooter extends Command {

  public Shooter shooter;

  public LoadPieceShooter() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    shooter.setShooterIndexerSpeed(Constants.Shooter.LOADING_INDEXER_SPEED);
  }

  @Override
  public void execute() {
    //if (shooter.getShooterIndexerBeambreak1()) {
    //  shooter.setShooterIndexerSpeed(Constants.Shooter.BEAMBREAK1_INDEXER_SPEED);
    //}
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterIndexerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;//shooter.getShooterIndexerBeambreak2();
  }
}
