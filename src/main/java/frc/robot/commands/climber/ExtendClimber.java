// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetClimberToPosition;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendClimber extends ParallelCommandGroup {

  public ExtendClimber() {
    addCommands(new SetClimberToPosition(ClimbModule.LEFT, ClimberConstants.MAX_POSITION),
        new SetClimberToPosition(ClimbModule.RIGHT, ClimberConstants.MAX_POSITION));
  }
}
