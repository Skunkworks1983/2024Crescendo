// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShootWhenReady extends Command {

  private Shooter shooter;
  private Indexer indexer;
  double angleError;
  int atSpeedCount = 0;
  int atPivotSetpointCount = 0;
  int minAtSpeedCount = 13;
  double timeAtInit;
  Timer timer;

  public ShootWhenReady() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    // only reads the flywheel, so it doesn't require the flywheel
    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
    timer = new Timer();
  }

  @Override
  public void initialize() {
    System.out.println("Shoot When Ready Command Initialize");
    atSpeedCount = 0;
    atPivotSetpointCount = 0;
    timeAtInit = timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    if (Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR) {
      atSpeedCount++;
    } else {
      atSpeedCount = 0;
    }

    if (shooter.isPivotAtSetpoint()) {
      atPivotSetpointCount++;
    } else {
      atPivotSetpointCount = 0;
    }

    if ((atSpeedCount > minAtSpeedCount && shooter.isFlywheelSpiningWithSetpoint
        && atPivotSetpointCount > Constants.Shooter.SHOOTER_ANGLE_WAIT_TICKS)
        || timer.getFPGATimestamp() - timeAtInit >= Constants.Shooter.SHOOT_WHEN_READY_SECONDS_BEFORE_SHOOT) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
    // SmartDashboard.putNumber("BSFlywheelerror", shooter.getFlywheelError());
    // SmartDashboard.putNumber("BSShooterPivotError", shooter.getShooterPivotError());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerMotorCoastMode();
    indexer.setIndexerCoastMode();
    System.out.println("Shooter speed setpoint: " + shooter.getFlywheelSetpoint() + ", velocity: "
        + shooter.getFlywheelVelocity() + ", error: " + shooter.getFlywheelError());

    /*
     * Pose2d robotPose = drivebase.getRobotPose(); */
    System.out.println("Shooter angle setpoint: " +
    shooter.getShooterSetpoint() + ", position: " + shooter.getShooterPivotRotationInDegrees() +
    ", error: " + shooter.getShooterPivotError()); 
    /*System.out.println("Odometry position X: " +
    robotPose.getX()); System.out.println("Odometry position Y: " + robotPose.getY());
    System.out.println("Odometry position Angle: " + robotPose.getRotation());*/
     
    System.out.println("Shoot When Ready Command End interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("BSShooterPivotError", shooter.getShooterPivotError());
    SmartDashboard.putNumber("BSShootFlywheelError", shooter.getFlywheelError());
    SmartDashboard.putNumber("BSShooter Pivot Setpoint", shooter.getPivotSetPoint());

    return !shooter.getShooterIndexerBeambreak1() && !shooter.getShooterIndexerBeambreak2()
        && !indexer.getBeamBreakSensor();
  }
}
