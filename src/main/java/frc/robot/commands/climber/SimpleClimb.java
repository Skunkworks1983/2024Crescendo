// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

public class SimpleClimb extends Command {
    Climber climber;

    public SimpleClimb() {
        climber = Climber.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SimpleClimb command started");
        climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.MIN_POSITION);
        climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.MIN_POSITION);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("SimpleClimb command ended");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(climber.getClimberPostition(ClimbModule.LEFT)
                - ClimberConstants.MIN_POSITION) < ClimberConstants.CLIMBER_POSITION_TOLERANCE
                && Math.abs(climber.getClimberPostition(ClimbModule.RIGHT)
                        - ClimberConstants.MIN_POSITION) < ClimberConstants.CLIMBER_POSITION_TOLERANCE) {
            return true;
        }
        return false;
    }

}
