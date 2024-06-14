// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterDiagnostic extends Command {

  Shooter shooter;
  SmartDashboardOutput[] diagnosticResults;
  
  public ShooterDiagnostic() {
    diagnosticResults = new SmartDashboardOutput[] {
      new SmartDashboardOutput("Shooter Flywheel", false),
      new SmartDashboardOutput("Shooter Pivot", false),
      new SmartDashboardOutput("Shooter Trigger", false)
    };

    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(.1);
    shooter.setPivotAngleAndSpeed(new Rotation2d(.1));
    shooter.setShooterIndexerSpeed(.1);
  }

  @Override
  public void execute() {
   
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Shooter Flywheel", flywheelDiagnosticResult);
    SmartDashboard.putBoolean("Shooter Pivot", );
    SmartDashboard.putBoolean("Shooter Trigger");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
