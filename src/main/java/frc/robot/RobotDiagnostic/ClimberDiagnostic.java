// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.subsystems.Climber;

public class ClimberDiagnostic extends Command {

  Climber climber;
  Map<String, Runnable> diagnosticOutputs = new HashMap<String, Runnable>();

  public ClimberDiagnostic() {
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  @Override
  public void initialize() {

    diagnosticOutputs.put("Left Climber", () -> SmartDashboard.putBoolean("Left Climber", false));
    diagnosticOutputs.put("Right Climber", () -> SmartDashboard.putBoolean("Right Climber", false));

    climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.DIAGNOSTIC_MOVEMENT);
    climber.setClimberPosition(ClimbModule.RIGHT, ClimberConstants.DIAGNOSTIC_MOVEMENT);
  }

  @Override
  public void execute() {
    if (climber.getClimberMotorVelocity(ClimbModule.LEFT) > 0.0) {
      diagnosticOutputs.put("Left Climber", () -> SmartDashboard.putBoolean("Left Climber", true));
    }

    if (climber.getClimberMotorVelocity(ClimbModule.RIGHT) > 0.0) {
      diagnosticOutputs.put("Right Climber", () -> SmartDashboard.putBoolean("Right Climber", true));
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberPosition(ClimbModule.LEFT, ClimberConstants.MIN_POSITION);
    climber.setClimberPosition(ClimbModule.RIGHT, ClimberConstants.MIN_POSITION);

    DiagnosticOutputs.addResults(diagnosticOutputs);
  }
}
