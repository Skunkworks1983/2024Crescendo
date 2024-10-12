// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DrivebaseDiagnostic extends Command {

  Drivebase drivebase;
  Map<String, Runnable> diagnosticOutputs = new HashMap<String, Runnable>();

  public DrivebaseDiagnostic() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

    diagnosticOutputs.put("FL Drive", () -> SmartDashboard.putBoolean("FL Drive", false));
    diagnosticOutputs.put("FL Turn", () -> SmartDashboard.putBoolean("FL Turn", false));
    diagnosticOutputs.put("FR Drive", () -> SmartDashboard.putBoolean("FR Drive", false));
    diagnosticOutputs.put("FR Turn", () -> SmartDashboard.putBoolean("FR Turn", false));
    diagnosticOutputs.put("BL Drive", () -> SmartDashboard.putBoolean("BL Drive", false));
    diagnosticOutputs.put("BL Turn", () -> SmartDashboard.putBoolean("BL Turn", false));
    diagnosticOutputs.put("BR Drive", () -> SmartDashboard.putBoolean("BR Drive", false));
    diagnosticOutputs.put("BR Turn", () -> SmartDashboard.putBoolean("BR Turn", false));

    // Set each of the drivebase motors to a slow speed.
    drivebase.setSwerveModuleMotorSpeeds(
        new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1),
            new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1)});
  }

  @Override
  public void execute() {
    SwerveModuleSpeeds[] speeds = drivebase.getSwerveModuleMotorSpeeds();

    if (speeds[0].driveSpeed != 0.0) {
      diagnosticOutputs.put("FL Drive", () -> SmartDashboard.putBoolean("FL Drive", true));
    }

    if (speeds[0].turnSpeed != 0.0) {
      diagnosticOutputs.put("FL Turn", () -> SmartDashboard.putBoolean("FL Turn", true));
    }

    if (speeds[1].driveSpeed != 0.0) {
      diagnosticOutputs.put("FR Drive", () -> SmartDashboard.putBoolean("FR Drive", true));
    }

    if (speeds[1].turnSpeed != 0.0) {
      diagnosticOutputs.put("FR Turn", () -> SmartDashboard.putBoolean("FR Turn", true));
    }

    if (speeds[2].driveSpeed != 0.0) {
      diagnosticOutputs.put("BL Drive", () -> SmartDashboard.putBoolean("BL Drive", true));
    }

    if (speeds[2].turnSpeed != 0.0) {
      diagnosticOutputs.put("BL Turn", () -> SmartDashboard.putBoolean("BL Turn", true));
    }

    if (speeds[3].driveSpeed != 0.0) {
      diagnosticOutputs.put("BR Drive", () -> SmartDashboard.putBoolean("BR Drive", true));
    }

    if (speeds[3].turnSpeed != 0.0) {
      diagnosticOutputs.put("BR Turn", () -> SmartDashboard.putBoolean("BR Turn", true));
    }

  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setSwerveModuleMotorSpeeds(new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(0, 0),
        new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0),});

    DiagnosticOutputs.addResults(diagnosticOutputs);
    System.out.println("Drivebase Diagnostic End");
  }
}
