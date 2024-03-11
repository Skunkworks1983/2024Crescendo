// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ManualMoveClimber extends Command {

  Climber climber;
  ClimbModule module;
  double percentOutput;

  public ManualMoveClimber(ClimbModule module, double percentOutput) {
    this.module = module;
    this.percentOutput = percentOutput;
    climber = Climber.getInstance();

    if (module == ClimbModule.LEFT) {
      addRequirements(SubsystemGroups.getInstance(Subsystems.CLIMBER_LEFT));
    } else if (module == ClimbModule.RIGHT) {
      addRequirements(SubsystemGroups.getInstance(Subsystems.CLIMBER_RIGHT));
    } else {
      addRequirements(SubsystemGroups.getInstance(Subsystems.CLIMBER_RIGHT),
          SubsystemGroups.getInstance(Subsystems.CLIMBER_LEFT));
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Move Climber Command Initialize");
    climber.setClimberOutput(module, percentOutput);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberOutput(module, 0);
    climber.setBrakeMode(module);
    System.out.println("Manual Move Climber Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
