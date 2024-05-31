// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterDiagnostic extends Command {

  Shooter shooter;
  
  public ShooterDiagnostic() {

    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setShooterIndexerSpeed();
    shooter.
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
