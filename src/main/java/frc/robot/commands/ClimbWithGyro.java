// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;

public class ClimbWithGyro extends Command {
  /** Creates a new ClimbWithGyro. */
  Climber climber;
  Drivebase drivebase;
  double leftOutput, rightOutput;

  public ClimbWithGyro() {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = Climber.getInstance();
    drivebase = Drivebase.getInstance();
    leftOutput = 0;
    rightOutput = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = drivebase.getGyroRoll();

    leftOutput = ClimberConstants.BASE_SPEED + roll / 100;
    rightOutput = ClimberConstants.BASE_SPEED - roll / 100;

    climber.setLeftClimberOutput(leftOutput);
    climber.setRightClimberOutput(rightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftBrakeMode();
    climber.setRightBrakeMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Checks if one of the climbers is at the minimun position, if yes, it ends the command
    if ((Math
        .abs(climber.getLeftClimberPostition()
            - ClimberConstants.CLIMBER1_POSITION_MIN) < ClimberConstants.CLIMBER_END_TOLERANCE
        || Math.abs(climber.getRightClimberPosition()
            - ClimberConstants.CLIMBER2_POSITION_MIN) < ClimberConstants.CLIMBER_END_TOLERANCE)
        /*&& Math.abs(drivebase.getGyroRoll()) < ClimberConstants.CLIMBER_END_ROLL_TOLERANCE*/) {
      return true;
    }
    return false;
  }
}
