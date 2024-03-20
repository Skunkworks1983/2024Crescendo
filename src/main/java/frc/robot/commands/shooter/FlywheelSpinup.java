// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class FlywheelSpinup extends Command {

  private Shooter shooter;

  public FlywheelSpinup() {
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_FLYWHEEL));
  }


  @Override
  public void initialize() {
    System.out.println("Flywheel Spinup Command Initialize");
  }


  @Override
  public void execute() {
    shooter.setFlywheelSpeed(Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED);
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
