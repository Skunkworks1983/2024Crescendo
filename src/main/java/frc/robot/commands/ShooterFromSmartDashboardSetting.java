package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterFromSmartDashboardSetting extends Command {

  private Shooter shooter;
  double timeAtStart;
  Rotation2d shooterAngle;
  private Indexer indexer;

  public ShooterFromSmartDashboardSetting() {
    SmartDashboard.putNumber("flywheelSpeed", 0.0);
    SmartDashboard.putNumber("shooterAngle", 0.0);
    indexer = Indexer.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    timeAtStart=Timer.getFPGATimestamp();
    shooterAngle = new Rotation2d(SmartDashboard.getNumber("shooterAngle", 0.0));
    shooter.setFlywheelSetpoint(SmartDashboard.getNumber("flywheelSpeed", 0.0));
    System.out.println("Shooter from smart dashboard setting Command Initialize");
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);

    if ((Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR)
        && (Math.abs(shooter
            .getShooterPivotRotationInDegrees()) <= Constants.Shooter.MAX_SHOOTER_PIVOT_ANGLE_ERROR)
        && shooter.isFlywheelSpiningWithSetpoint) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelSetpoint(0.0);
    shooter.setPivotMotorPercentOutput(0.0);
    System.out.println("Shooter from smart dashboard setting Command End");
    System.out.println("Command took " + (Timer.getFPGATimestamp()-timeAtStart)+" seconds.");
    Pose2d a = Drivebase.getInstance().getRobotPose();
    System.out.println("Robot thought it was at the position: " + a.getX() + "," + a.getY());
  }

  @Override
  public boolean isFinished() {
    return !shooter.getShooterIndexerBeambreak2() && !shooter.getShooterIndexerBeambreak1();
  }
}
