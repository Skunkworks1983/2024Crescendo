// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class SmartClimb extends SequentialCommandGroup {
  Climber climber;

  public SmartClimb() {
    climber = Climber.getInstance();

    // Sequential Command group with LowerClimberToChain and ClimbWithGyro.
    addCommands(new LowerClimberToChain(), new ClimbWithGyro());
  }
}
