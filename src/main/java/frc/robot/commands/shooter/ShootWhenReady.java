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

  public ShootWhenReady() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    drivebase = Drivebase.getInstance();
    // only reads the flywheel, so it doesn't require the flywheel
    addRequirements(SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(shooter.flywheelSetpointMPS);
    System.out.println("Shoot When Ready Command Initialize");
  }

  @Override
  public void execute() {
    if (Math.abs(shooter.getFlywheelError()) <= Constants.Shooter.MAX_FLYWHEEL_ERROR
        && shooter.isFlywheelSpiningWithSetpoint) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerMotorCoastMode();
    indexer.setIndexerCoastMode();
    Pose2d robotPose = drivebase.getRobotPose();
    // "*" indicates data to be put in a csv.
    System.out.println(
      "*,Shooter angle setpoint," + shooter.getShooterSetpoint() + 
    "\n*, position, "
        + shooter.getShooterPivotRotationInDegrees() + 
        "\n*,error: "+ shooter.getShooterPivotError()+
        "\n*,Shooter speed setpoint," + shooter.getFlywheelSetpoint() + 
    "\n *, velocity, "+ shooter.getFlywheelVelocity() + ", error," + shooter.getFlywheelError() + 
    "\n*,Odometry position X, " + robotPose.getX()+
    "\n*,Odometry position Y," + robotPose.getY() +
    "\n*,Odometry position Angle," + robotPose.getRotation() +
    "\n*,Shoot When Ready Command End"+
    "\n*,NEXT");
  }

  @Override
  public boolean isFinished() {
    return !shooter.getShooterIndexerBeambreak2() && !shooter.getShooterIndexerBeambreak1();
  }
}
