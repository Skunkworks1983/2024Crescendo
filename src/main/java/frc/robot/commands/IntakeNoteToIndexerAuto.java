// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class IntakeNoteToIndexerAuto extends Command {
  private final Collector collector;
  private final Indexer indexer;

  /** Creates a new RunCollectorAndIndexer. */
  public IntakeNoteToIndexerAuto() {
    collector = Collector.getInstance();
    indexer = Indexer.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setPercentOutput(1);
    indexer.setPercentOutput(1);
    System.out.println("Intake Note to Indexer Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setIntakeCoastMode();
    collector.setPercentOutput(0);
    indexer.setPercentOutput(0);
    System.out.println("Intake Note to Indexer Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.getBeamBreakSensor();

  }
}
