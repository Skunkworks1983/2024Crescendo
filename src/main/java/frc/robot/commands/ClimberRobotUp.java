// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

//This is a stub command
public class ClimberRobotUp extends Command {
    private Climber climber;
    public ClimberRobotUp(){
        climber = Climber.getInstance();
        addRequirements(Climber.getInstance());
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    climber.setClimberPosition(true, ClimberConstants.CLIMBER1_POSITION_MIN);
    climber.setClimberPosition(false, ClimberConstants.CLIMBER2_POSITION_MIN);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(Math.abs(climber.getClimber1Postition() - ClimberConstants.CLIMBER1_POSITION_MIN) < ClimberConstants.CLIMBER_TOLERANCE){
        
        return false;
        }
    }

}
  