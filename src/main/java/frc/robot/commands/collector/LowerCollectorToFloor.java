// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class LowerCollectorToFloor extends Command {

  Collector collector;
  
  public LowerCollectorToFloor() {
    collector = Collector.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.COLLECTOR_PIVOT));
  }

  @Override
  public void initialize() {
    System.out.println("LowerCollectorToFloor command started");
    collector.setCollectorGoal(Constants.Collector.COLLECTOR_FLOOR_POS);

    // Don't continue until the collect has reached the minimum
    while (!(collector.isAtCollectorGoal())) {
    }

    collector.setPivotPercentOutput(.05);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("PutCollectorToFloor command ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
