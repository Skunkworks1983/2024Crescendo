// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ShootingTargetHelper;

public class ShooterToSpeaker extends Command {

  Translation3d noteInitialVelocity;
  public Shooter shooter = Shooter.getInstance();
  public ShootingTargetHelper shootingTargetHelper;

  public ShooterToSpeaker() {
    shootingTargetHelper = new ShootingTargetHelper();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation3d speakerTarget =
        new Translation3d(Constants.SpeakerTargetingMath.SPEAKER_POSITION.getX(),
            Constants.SpeakerTargetingMath.SPEAKER_POSITION.getY(),
            Constants.SpeakerTargetingMath.SPEAKER_POSITION.getZ());

    shooter.setShooterAngle(
        new Rotation2d(shootingTargetHelper.calculateShooterAngle(speakerTarget)),
        shootingTargetHelper.calculateFlywheelSpeed(speakerTarget));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
