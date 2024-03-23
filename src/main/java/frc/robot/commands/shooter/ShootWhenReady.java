// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShootWhenReady extends Command {

  private Shooter shooter;
  private Indexer indexer;
  private Drivebase drivebase;
  double angleError;
  int atSpeedCount = 0;
  int minAtSpeedCount = 13;

  public ShootWhenReady() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    drivebase = Drivebase.getInstance();
    // only reads the flywheel, so it doesn't require the flywheel
    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  @Override
  public void initialize() {
    System.out.println("Shoot When Ready Command Initialize");
    atSpeedCount = 0;
  }

  @Override
  public void execute() {
    if(Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR){
       atSpeedCount++;
    }
    else{
      atSpeedCount = 0;
    }

    if (atSpeedCount > minAtSpeedCount && shooter.isFlywheelSpiningWithSetpoint) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerMotorCoastMode();
    indexer.setIndexerCoastMode();
    System.out.println("Shooter speed setpoint: " + shooter.getFlywheelSetpoint() + ", velocity: "
        + shooter.getFlywheelVelocity() + ", error: " + shooter.getFlywheelError());
    /*
    Pose2d robotPose = drivebase.getRobotPose();
    System.out.println("Shooter angle setpoint: " + shooter.getShooterSetpoint() + ", position: "
        + shooter.getShooterPivotRotationInDegrees() + ", error: "
        + shooter.getShooterPivotError());
    System.out.println("Odometry position X: " + robotPose.getX());
    System.out.println("Odometry position Y: " + robotPose.getY());
    System.out.println("Odometry position Angle: " + robotPose.getRotation()); */
    System.out.println("Shoot When Ready Command End interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    return !shooter.getShooterIndexerBeambreak1() && !shooter.getShooterIndexerBeambreak2()
        && !indexer.getBeamBreakSensor();
  }
}
