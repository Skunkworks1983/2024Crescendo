// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

public class SetClimberToPosition extends Command {

  Climber climber;
  ClimbModule module;
  double position;

  public SetClimberToPosition(ClimbModule module, double position) {
    this.position = position;
    this.module = module;
    climber = Climber.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetClimberToPosition command started");
    climber.setClimberPosition(module, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberOutput(module, 0);
    climber.setBrakeMode(module);
    SmartDashboard.putBoolean("SetClimberToPosition " + module, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.atPositionSetpoint(position, module);
  }
}
