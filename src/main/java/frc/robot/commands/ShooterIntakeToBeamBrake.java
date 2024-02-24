// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterIntakeToBeamBrake extends Command {
  private final Shooter shooter;
  /** Creates a new ShooterIntakeToBeamBrake. */

  @Deprecated
  public ShooterIntakeToBeamBrake() {
    shooter = Shooter.getInstance();
    //this is a duplicate of LoadPieceShooter, do not use
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterIndexerSpeed(Constants.Shooter.SHOOTER_INDEXER_SPEED_TO_BEAM_BRAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getShooterIndexerBeambreak1() == shooter.getShooterIndexerBeambreak2();
  }
}
