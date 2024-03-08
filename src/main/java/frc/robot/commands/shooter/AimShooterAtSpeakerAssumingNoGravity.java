// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;
import frc.robot.utils.ShooterAimUtils;

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

    //Flip position if on red side of the field
    double positionShiftOnRedSide = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      positionShiftOnRedSide = Math.abs(
          Constants.Targeting.FieldTarget.SPEAKER.get().get().getX() - Constants.FIELD_X_LENGTH / 2)
          * 2;
    }

    target = new Translation3d(Constants.Targeting.FieldTarget.SPEAKER.get().get().getX() + positionShiftOnRedSide,
        Constants.Targeting.FieldTarget.SPEAKER.get().get().getY(),

        Constants.Targeting.FieldTarget.SPEAKER.get().get().getZ());

    System.out.println("Aim Shooter at Speaker Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d shooterPivot =
        ShooterAimUtils.calculatePivotPositionFieldRelative(drivebase.getRobotHeading(),
            Units.degreesToRadians(shooter.getShooterPivotRotationInDegrees()),
            drivebase.getRobotPose().getTranslation());

    Translation2d diffrenceInPosition =
        new Translation2d(target.getX() - shooterPivot.getX(), target.getY() - shooterPivot.getY());
    // 90 - theta is neccecary to convert from the system in which forward is 90 and
    // up is 0 to the
    // system in which 0 is forward and 90 is upward.
    Rotation2d shooterRotation =
        new Rotation2d((Math.PI / 2.0)
            - Math.atan2(target.getZ() - shooterPivot.getZ(),
                diffrenceInPosition.getNorm()
                    - Constants.Shooter.ROBOT_RELATIVE_PIVOT_POSITION.getX())
            + Units.degreesToRadians(Constants.Shooter.AUTOAIMING_OFFSET));

    //Ensures that setpoint is not outside the range of rotations the shooter pivot can make
    if (shooterRotation.getDegrees() >= 90.0) {
      shooterRotation = new Rotation2d(Math.PI / 2.0);
    } else if (shooterRotation.getDegrees() <= Constants.Shooter.SHOOTER_RESTING_POSITION
        .getDegrees()) {
      shooterRotation = Constants.Shooter.SHOOTER_RESTING_POSITION;
    }

    shooter.setPivotAngleAndSpeed(shooterRotation);
    shooter.setFlywheelSetpoint(Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Aim Shooter at Speaker Command End");
    shooter.setFlywheelMotorCoastMode();
    drivebase.setFieldTarget(FieldTarget.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
