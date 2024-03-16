// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class LoadPieceShooter extends Command {

  private Shooter shooter;
  private Indexer indexer;
  private Collector collector;
  private boolean canLoadPiece;
  private boolean beambreak1Tripped;
  private boolean initialSpeedSet;


  public LoadPieceShooter() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    collector = Collector.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  @Override
  public void initialize() {
    canLoadPiece = shooter.canLoadPiece();

    // This is set to false so we update the speeds if this command is run and the note only
    // triggers the first beambreak.
    beambreak1Tripped = false;
    initialSpeedSet = false;
    System.out.println("Load Piece Shooter Command Initialize");
  }

  @Override
  public void execute() {
    if (!canLoadPiece) {
      canLoadPiece = shooter.canLoadPiece();
    }

    if (!canLoadPiece) {
      return;
    }

    if (!initialSpeedSet && !shooter.getShooterIndexerBeambreak1()) {
      shooter.setPivotMotorPercentOutput(Constants.Shooter.SHOOTER_PIVOT_SLOW_DOWN_LOAD_PEICE);
      shooter.setIndexerPercentOutput(Constants.Shooter.LOADING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      if (!collector.isStowed()) {
        collector.setPercentOutput(Constants.Collector.COLLECTOR_MANUAL_PERCENT_OUTPUT);
      }
      initialSpeedSet = true;
      System.out.println("Load Piece Shooter initialSpeedSet");
    }

    if (!beambreak1Tripped && shooter.getShooterIndexerBeambreak1()) {
      shooter.setPivotMotorPercentOutput(Constants.Shooter.SHOOTER_PIVOT_SLOW_DOWN_LOAD_PEICE);
      shooter.setIndexerPercentOutput(Constants.Shooter.BEAMBREAK1_INDEXER_SPEED);

      // when enabled, if only the first beambreak is tripped, this will be the only thing setting
      // speeds on the indexer, and we need to have the indexer and the shooter indexer running then
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      collector.setIntakeCoastMode();
      beambreak1Tripped = true;
      System.out.println("Load Piece Shooter beam break slowed");
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterIndexerSpeed(0);
    indexer.setIndexerCoastMode();
    collector.setIntakeCoastMode();
    shooter.setPivotMotorPercentOutput(0);
    System.out.println("Load Piece Shooter Command End");
  }

  @Override
  public boolean isFinished() {
    return shooter.getShooterIndexerBeambreak2();
  }
}
