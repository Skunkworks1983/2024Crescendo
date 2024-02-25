// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FlywheelSpinup extends Command {

  public Shooter shooter;

  public FlywheelSpinup() {
    shooter = Shooter.getInstance();
  }


  @Override
  public void initialize() {
    System.out.println("Flywheel Spinup Command Initialize");
  }


  @Override
  public void execute() {
    shooter.setFlywheelSpeed(shooter.flywheelSetpointMPS);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelMotorCoastMode();
    System.out.println("Flywheel Spinup Command End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
