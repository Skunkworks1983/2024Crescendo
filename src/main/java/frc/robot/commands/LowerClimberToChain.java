// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.CLIMB_MODULE;
import frc.robot.subsystems.Climber;

public class LowerClimberToChain extends Command {
  /** Creates a new LowerTellCurrent. */
  Climber climber;
  public LowerClimberToChain() {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = Climber.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getClimberTorque(CLIMB_MODULE.LEFT) < ClimberConstants.CLIMBER_CHAIN_TORQUE){
    climber.setClimberOutput(CLIMB_MODULE.LEFT, ClimberConstants.BASE_PULL_SPEED);
    }
    if(climber.getClimberTorque(CLIMB_MODULE.LEFT) < ClimberConstants.CLIMBER_CHAIN_TORQUE)
    climber.setClimberOutput(CLIMB_MODULE.RIGHT, ClimberConstants.BASE_PULL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(climber.getClimberTorque(CLIMB_MODULE.LEFT) < ClimberConstants.CLIMBER_CHAIN_TORQUE &&
     climber.getClimberTorque(CLIMB_MODULE.RIGHT) < ClimberConstants.CLIMBER_CHAIN_TORQUE);
     }
}
