// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class RunCollectorAndIndexer extends Command {
  private final Collector collector;
  private final Indexer indexer;

  /** Creates a new RunCollectorAndIndexer. */
  public RunCollectorAndIndexer() {
    collector = Collector.getInstance();
    indexer = Indexer.getInstance();
    
    addRequirements(indexer, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.intakeNotes(Constants.Collector.NOTE_INTAKE_SPEED);
    indexer.setSpeedIndexer(Constants.IndexerConstants.INDEXER_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setIntakeCoastMode();
    indexer.setSpeedIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.getBeamBreakSensor();
  }
}
