// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.Map;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DrivebaseDiagnostic extends Command {

  Drivebase drivebase;
  Map<String, Runnable> diagnosticResults;

  public DrivebaseDiagnostic() {

    diagnosticResults = Map.ofEntries(
      Map.entry("FL Drive", () -> SmartDashboard.putBoolean("FL Drive", false)),
      Map.entry("FL Turn", () -> SmartDashboard.putBoolean("FL Turn", false)),
      Map.entry("FR Drive", () -> SmartDashboard.putBoolean("FR Drive", false)),
      Map.entry("FR Turn", () -> SmartDashboard.putBoolean("FR Turn", false)),
      Map.entry("BL Drive", () -> SmartDashboard.putBoolean("BL Drive", false)),
      Map.entry("BL Turn", () -> SmartDashboard.putBoolean("BL Turn", false)),
      Map.entry("BR Drive", () -> SmartDashboard.putBoolean("BR Drive", false)),
      Map.entry("BR Turn", () -> SmartDashboard.putBoolean("BR Turn", false))
    );

    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

    // Set each of the drivebase motors to a slow speed.
    drivebase.setSwerveModuleMotorSpeeds(
        new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1),
            new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1)});
  }

  @Override
  public void execute() {
    for()
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setSwerveModuleMotorSpeeds(new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(0, 0),
        new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0),});

    timer.stop();

    DiagnosticResults.addResults(diagnosticResults);
    System.out.println("Drivebase Diagnostic End");
  }
}
