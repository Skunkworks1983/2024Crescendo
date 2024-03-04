// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.ejml.equation.MatrixConstructor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ShooterAimUtils;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class AimShooterAtSpeakerAssumingNoGravity extends Command {
  Shooter shooter;
  Translation3d target;
  Drivebase drivebase;

  /** Creates a new AimShooterAtSpeakerAssumingNoGravity. */
  public AimShooterAtSpeakerAssumingNoGravity() {
    shooter = Shooter.getInstance();
    drivebase = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = Constants.Targeting.FieldTarget.SPEAKER.get().get();
    System.out.println("Aim Shooter at Speaker Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d shooterPivot =
        ShooterAimUtils.calculatePivotPositionFieldReletive(drivebase.getGyroAngle(),
            Units.degreesToRadians(shooter.getShooterPivotRotationInDegrees()), drivebase.getRobotPose().getTranslation());

    Translation2d diffrenceInPosition =
        new Translation2d(target.getX() - shooterPivot.getX(), target.getY() - shooterPivot.getY());
        
    // 90 - theta is neccecary to convert from the system in which forward is 90 and up is 0 to the
    // system in which 0 is forward and 90 is upward.
    Rotation2d shooterRotation = new Rotation2d((Math.PI/2) - Math.atan2(target.getZ() - shooterPivot.getZ(),
        diffrenceInPosition.getNorm() - Constants.Shooter.ROBOT_RELATIVE_PIVOT_POSITION.getX()));

    shooter.setPivotAngleAndSpeed(shooterRotation);
    shooter.setFlywheelSetpoint(Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Aim Shooter at Speaker Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
