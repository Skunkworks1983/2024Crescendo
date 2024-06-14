// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

public class ClimberDiagnostic extends Command {

  Climber climber;
  SmartDashboardOutput[] diagnosticResults;

  public ClimberDiagnostic() {

    diagnosticResults = new SmartDashboardOutput[] {
      new SmartDashboardOutput("Left Climber", false),
      new SmartDashboardOutput("Right Climber", false)
    };

    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.DIAGNOSTIC_MOVEMENT);
    climber.setClimberPosition(ClimbModule.RIGHT, ClimberConstants.DIAGNOSTIC_MOVEMENT);
  }

  @Override
  public void execute() {
    if (climber.getClimberMotorVelocity(ClimbModule.LEFT) > .3) {
      diagnosticResults[0].instance = new SmartDashboardOutput("Left Climber", true);
    }

    if (climber.getClimberMotorVelocity(ClimbModule.RIGHT) > .3) {
      diagnosticResults[1].instance = new SmartDashboardOutput("Right Climber", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.MIN_POSITION);
    climber.setClimberPosition(ClimbModule.RIGHT, ClimberConstants.MIN_POSITION);

    DiagnosticResults.addResults(diagnosticResults);
  }
}
