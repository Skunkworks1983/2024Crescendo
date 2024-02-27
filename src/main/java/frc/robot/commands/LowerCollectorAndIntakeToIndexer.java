// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.ShooterToStow;

public class LowerCollectorAndIntakeToIndexer extends ParallelCommandGroup {

  public LowerCollectorAndIntakeToIndexer() {

    // Thomas said this should not be used, double check with drive team before using this ever
    addCommands(new LowerCollector(), new IntakeNoteToIndexer(), new ShooterToStow());
  }
}
