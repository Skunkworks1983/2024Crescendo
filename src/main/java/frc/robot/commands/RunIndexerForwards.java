// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;

public class RunIndexerForwards extends Command {
  private Indexer indexer;

  /** Creates a new RunIndexer. */
  public RunIndexerForwards() {
    indexer = Indexer.getInstance();
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setSpeedIndexer(Constants.IndexerConstants.INDEXER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: uncomment code after neccecary hardware is complete.
    return false;// indexer.getBeamBreakSensor();
  }
}
