// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

// This is a stub command
public class SetClimberToPosition extends Command {

  Climber climber;
  double position;
  ClimbModule module;
  

  public SetClimberToPosition(ClimbModule module, double position) {
    this.position = position;
    this.module = module;
    climber = Climber.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ExtendClimber command stared");
    climber.setClimberPosition(module, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ExtendClimber command ended");
    climber.setClimberOutput(module, 0);
    climber.setBrakeMode(module);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.atPositionSetpoint(module, position)) {
      return true;
    }
    return false;
  }
}
