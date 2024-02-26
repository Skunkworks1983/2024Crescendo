// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

// this command lowers the climber arms are in contact with the chain
public class LowerClimberToChain extends Command {
  /** Creates a new LowerClimberToChain. */
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
    if (climber.getClimberTorque(ClimbModule.LEFT) < ClimberConstants.CLIMBER_CHAIN_TORQUE) {
      climber.setClimberOutput(ClimbModule.LEFT, ClimberConstants.BASE_PULL_SPEED);
    } else {
      climber.setClimberOutput(ClimbModule.LEFT, 0);
      climber.setBrakeMode(ClimbModule.LEFT);
    }

    if (climber.getClimberTorque(ClimbModule.RIGHT) < ClimberConstants.CLIMBER_CHAIN_TORQUE) {
      climber.setClimberOutput(ClimbModule.RIGHT, ClimberConstants.BASE_PULL_SPEED);
    } else {
      climber.setClimberOutput(ClimbModule.RIGHT, 0);
      climber.setBrakeMode(ClimbModule.RIGHT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(climber.getClimberTorque(ClimbModule.LEFT) < ClimberConstants.CLIMBER_CHAIN_TORQUE &&
     climber.getClimberTorque(ClimbModule.RIGHT) < ClimberConstants.CLIMBER_CHAIN_TORQUE);
     }
}
