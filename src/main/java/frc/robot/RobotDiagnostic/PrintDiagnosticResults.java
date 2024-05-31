// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;

public class PrintDiagnosticResults extends InstantCommand {

  Drivebase drivebase;

  public PrintDiagnosticResults() {
    drivebase = Drivebase.getInstance();
  }

  @Override
  public void initialize() {
    Map<String, Boolean> diagnosticResults = drivebase.getDiagnosticResults();

    for (Map.Entry<String, Boolean> result : diagnosticResults.entrySet()) {
      SmartDashboard.putBoolean(result.getKey(), result.getValue());
    }
  }
}
