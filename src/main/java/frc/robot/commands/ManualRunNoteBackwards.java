// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ManualRunNoteBackwards extends Command {
  private Indexer indexer;
  private Shooter shooter;
  private Collector collector;

  /** Creates a new RunIndexerBackwards. */
  public ManualRunNoteBackwards() {
    indexer = Indexer.getInstance();
    shooter = Shooter.getInstance();
    collector = Collector.getInstance();

    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setPercentOutput(Constants.IndexerConstants.REVERSE_INDEXER_SPEED_PERCENT_OUTPUT);
    shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTER_MANUAL_INDEXER_BACKWARDS);
    
    System.out.println("Run Indexer Backwards Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(collector.isAtFloor()) {
      collector.setPercentOutput(Constants.Collector.REVERSE_COLLECTOR_SPEED);
      }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPercentOutput(0);
    collector.setPercentOutput(0);
    shooter.setIndexerPercentOutput(0);
    System.out.println("Run Indexer Backwards Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
