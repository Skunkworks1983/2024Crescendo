// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DrivebaseDiagnostic extends Command {

  Drivebase drivebase;
  boolean[] swerveModuleDrive;
  boolean[] swerveModuleSteer;
  Timer timer;
  String[] diagnosticNames = new String[] {"FL Drive", "FR Drive", "BL Drive", "BR Drive", "FL Turn", "FR Turn", "BL Turn", "BR Turn"};

  public DrivebaseDiagnostic() {
    timer = new Timer();
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    timer.restart();
    drivebase.setSwerveModuleMotorSpeeds(
        new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1),
            new SwerveModuleSpeeds(.2, .1), new SwerveModuleSpeeds(.2, .1)});
  }

  @Override
  public void execute() {
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putBoolean(diagnosticNames[i],
          drivebase.areSwerveModulesRunning()[i].isDriveRunning);
    }

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putBoolean(diagnosticNames[i + 4],
          drivebase.areSwerveModulesRunning()[i].isTurnRunning);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setSwerveModuleMotorSpeeds(new SwerveModuleSpeeds[] {new SwerveModuleSpeeds(0, 0),
        new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0), new SwerveModuleSpeeds(0, 0),});

    timer.stop();

    for (int i = 0; i < 4; i++) {
      drivebase.addDiagnosticResult(diagnosticNames[i],
          drivebase.areSwerveModulesRunning()[i].isDriveRunning);
    }

    for (int i = 0; i < 4; i++) {
      drivebase.addDiagnosticResult(diagnosticNames[i + 4],
          drivebase.areSwerveModulesRunning()[i].isTurnRunning);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(20.0);
  }
}
