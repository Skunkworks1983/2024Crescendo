// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.PivotCommand;

public class FlywheelSpinup extends Command {

  public Shooter shooter = Shooter.getInstance();

  public FlywheelSpinup() {


  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {

    PivotCommand pivotArmCommand = shooter.getPivotArmCommand();

    if(pivotArmCommand == PivotCommand.shooterToStow) {
      shooter.setShooterSpeed(Constants.Shooter.STOW_FLYWHEEL_SPEED);
    }
    else if(pivotArmCommand == PivotCommand.shooterToAmp) {
      shooter.setShooterSpeed(Constants.Shooter.AMP_FLYWHEEL_SPEED);
    }
    else if(pivotArmCommand == PivotCommand.shooterToSpeaker) {
      //TODO, get math and stuff and set flywheel speed based on this
    }
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
