// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShootWhenReady extends Command {

  private Shooter shooter;

  public ShootWhenReady() {
    shooter = Shooter.getInstance();
    //only reads the flywheel, so it doesn't require the flywheel
    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  @Override
  public void initialize() {
    System.out.println("Shoot When Ready Command Initialize");
  }

  @Override
  public void execute() {
    if (Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR
        && shooter.isFlywheelSpiningWithSetpoint) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerMotorCoastMode();
    System.out.println("Shoot When Ready Command End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
