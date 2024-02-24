// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

public class AimShooterAtSpeakerAssumingNoGravity extends Command {
  Shooter shooter;

  /** Creates a new AimShooterAtSpeakerAssumingNoGravity. */
  public AimShooterAtSpeakerAssumingNoGravity() {
    shooter = Shooter.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d target = Constants.Targeting.FieldTarget.SPEAKER.get().get();

    // from center of robot to target -- not shooter pivot.
    Translation2d diffrenceInPosition = new Translation2d(
        target.getX() - Drivebase.getInstance().getRobotPose().getX(),
        target.getY() - Drivebase.getInstance().getRobotPose().getY());

    Rotation2d shooterRotation = new Rotation2d(
        Math.atan2(
            target.getZ() - Constants.Shooter.ROBOT_RELATIVE_PIVOT_POSITION.getZ(),
            diffrenceInPosition.getNorm() - Constants.Shooter.ROBOT_RELATIVE_PIVOT_POSITION.getX()));

    shooter.setShooterAngle(shooterRotation);
    shooter.setFlywheelSetpoint(Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
