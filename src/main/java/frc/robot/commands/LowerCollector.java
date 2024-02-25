// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;

public class LowerCollector extends Command {
  private final Collector collector;
  /** Creates a new LowerCollector. */
  public LowerCollector() {
    collector = Collector.getInstance();
    
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setCollectorPos(Constants.Collector.COLLECTOR_FLOOR_POS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setCollectorPivotVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check collector position
    return collector.isAtFloor();
  }
}
