// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.PivotPosition;

public class FlywheelSpinup extends Command {

  public Shooter shooter;

  public FlywheelSpinup() {
    shooter = Shooter.getInstance();
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {

    PivotPosition pivotPosition = shooter.getPivotArmCommand();

    if (pivotPosition == PivotPosition.SHOOTER_TO_STOW) {
      shooter.setShooterSpeed(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    } else if (pivotPosition == PivotPosition.SHOOTER_TO_STOW) {
      shooter.setShooterSpeed(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    }
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setShootMotorVelocity0();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
