// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class CollectorStow extends Command {
  private final Collector collector;

  /** Creates a new CollectorStow. */
  public CollectorStow() {
    this.collector = Collector.getInstance();

    addRequirements(SubsystemGroups.getInstance(Subsystems.COLLECTOR_PIVOT));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setCollectorGoal(Constants.Collector.COLLECTOR_STOW_POS);
    System.out.println("Collector Stow Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Collector Stow Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collector.isStowed();
  }
}
