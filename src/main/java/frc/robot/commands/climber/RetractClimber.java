// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;

public class RetractClimber extends ParallelCommandGroup {
  public RetractClimber() {
    addCommands(new SetClimberToPosition(ClimbModule.LEFT, ClimberConstants.MIN_POSITION),
        new SetClimberToPosition(ClimbModule.RIGHT, ClimberConstants.MIN_POSITION));
  }
}
