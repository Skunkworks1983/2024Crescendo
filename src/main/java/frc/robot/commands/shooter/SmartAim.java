// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ShooterAimUtils;

public class SmartAim extends Command {
  Shooter shooter;
  Drivebase drivebase;

  /** Creates a new SmartAim. */
  public SmartAim() {
    shooter = Shooter.getInstance();
    drivebase = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation3d pivotPositionFieldReletive =
        ShooterAimUtils.calculatePivotPositionFieldReletive(drivebase.getRobotHeading(),
            shooter.getShooterPivotRotation(), drivebase.getRobotPose().getTranslation());

    double flywheelSpeed =
        ShooterAimUtils.calculateIdealFlywheelSpeed(pivotPositionFieldReletive.toTranslation2d());
    shooter.setFlywheelSetpoint(flywheelSpeed);

    double shooterAngle = ShooterAimUtils
        .calculateIdealStationaryShooterPivotAngle(pivotPositionFieldReletive, flywheelSpeed);
    shooter.setShooterAngle(new Rotation2d(shooterAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
