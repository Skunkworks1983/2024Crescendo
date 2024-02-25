// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class LoadPieceShooter extends Command {

  private Shooter shooter;
  private Indexer indexer;
  private boolean canLoadPiece;
  private boolean beambreak1Tripped;
  private boolean beambreak2Tripped;


  public LoadPieceShooter() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
  }

  @Override
  public void initialize() {
    canLoadPiece = shooter.canLoadPiece();
    beambreak1Tripped = shooter.getShooterIndexerBeambreak1();
    beambreak2Tripped = shooter.getShooterIndexerBeambreak2();
  }

  @Override
  public void execute() {
    if (!canLoadPiece) {
      canLoadPiece = shooter.canLoadPiece();
    }

    if (!canLoadPiece) {
      return;
    }

    if (!shooter.getShooterIndexerBeambreak1() && !beambreak1Tripped) {
      shooter.setShooterIndexerSpeed(Constants.Shooter.LOADING_INDEXER_SPEED);
      indexer.setSpeedIndexer(Constants.Shooter.LOADING_INDEXER_SPEED);
      beambreak1Tripped = true;
    } 
    
    if(shooter.getShooterIndexerBeambreak1() && !beambreak2Tripped) {
      shooter.setShooterIndexerSpeed(Constants.Shooter.BEAMBREAK1_INDEXER_SPEED);
      indexer.setIndexerCoastMode();
      beambreak2Tripped = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterIndexerSpeed(0);
    indexer.setIndexerCoastMode();
  }

  @Override
  public boolean isFinished() {
    return shooter.getShooterIndexerBeambreak2();
  }
}
