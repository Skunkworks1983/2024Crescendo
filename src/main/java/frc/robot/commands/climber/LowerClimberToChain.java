// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

// this command lowers the climber arms are in contact with the chain
public class LowerClimberToChain extends Command {
  /** Creates a new LowerClimberToChain. */
  Climber climber;
  boolean isTorqueLeft = true;
  boolean isTorqueRight = true;

  public LowerClimberToChain() {
    climber = Climber.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("LowerClimberToChain command started");
    SmartDashboard.putBoolean("LowerClimberToChain", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isTorqueLeft = climber.getClimberTorque(ClimbModule.LEFT) > ClimberConstants.CLIMBER_CHAIN_TORQUE;
    isTorqueRight = climber.getClimberTorque(ClimbModule.RIGHT) > ClimberConstants.CLIMBER_CHAIN_TORQUE;

    if (!isTorqueLeft) {
      climber.setClimberOutput(ClimbModule.LEFT, ClimberConstants.BASE_PULL_SPEED);
    } else {
      climber.setClimberOutput(ClimbModule.LEFT, 0);
      climber.setBrakeMode(ClimbModule.LEFT);
    }

    if (!isTorqueRight) {
      climber.setClimberOutput(ClimbModule.RIGHT, ClimberConstants.BASE_PULL_SPEED);
    } else {
      climber.setClimberOutput(ClimbModule.RIGHT, 0);
      climber.setBrakeMode(ClimbModule.RIGHT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("LowerClimberToChain command ended");
    SmartDashboard.putBoolean("LowerClimberToChain", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isTorqueLeft && isTorqueRight);
  }
}
