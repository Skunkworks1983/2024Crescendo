package frc.robot.commands.shooter.tuningCommands;
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

public class smartDashboardAim extends Command {

  private Shooter shooter;
  double timeAtStart;
  Rotation2d shooterAngle;
  private Indexer indexer;

  int flywheelTicks;
  double flywheelSpeed;
  int pivotTicks;
  int ticksToShoot = 15;//move to constants

  public smartDashboardAim() {
    SmartDashboard.putNumber("flywheelSpeed", 0.0);
    SmartDashboard.putNumber("shooterAngle", 0.0);
    indexer = Indexer.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT), 
        SubsystemGroups.getInstance(Subsystems.SHOOTER_FLYWHEEL),
        SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
  }

  @Override
  public void initialize() {
    flywheelTicks=0;
    pivotTicks=0;
    timeAtStart=Timer.getFPGATimestamp();
    shooterAngle =Rotation2d.fromDegrees(SmartDashboard.getNumber("shooterAngle", 0.0));
    flywheelSpeed = SmartDashboard.getNumber("flywheelSpeed", 0.0);
    shooter.setFlywheelSpeed(SmartDashboard.getNumber("flywheelSpeed", 0.0));
    if (shooterAngle.getDegrees() > 90.0) {
      shooterAngle = Rotation2d.fromDegrees(90);
    } else if (shooterAngle.getDegrees() < Constants.Shooter.SHOOTER_RESTING_POSITION
        .getDegrees()) {
      shooterAngle = Constants.Shooter.SHOOTER_RESTING_POSITION;
    }

    System.out.println("Shooter from smart dashboard setting Command Initialize");
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);

    if ((Math.abs(shooter.getFlywheelVelocity()-flywheelSpeed) <= Constants.Shooter.MAX_FLYWHEEL_ERROR)){
      flywheelTicks++;
    }
    else{
      flywheelTicks=0;
    }

    if(shooter.isFlywheelSpiningWithSetpoint && (Math.abs(shooter.getShooterPivotRotationInDegrees()
    - shooterAngle.getDegrees()) <= .5)){
      pivotTicks++;
    }else{
      pivotTicks=0;
    }

    //System.out.println(flywheelTicks+","+ pivotTicks + " : " + shooter.getFlywheelError());
    //System.out.println(shooter.getFlywheelVelocity());
    if(pivotTicks>=ticksToShoot && flywheelTicks>=ticksToShoot){
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
      indexer.setPercentOutput(Constants.Shooter.SHOOTING_INDEXER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelMotorCoastMode();
    shooter.setIndexerMotorCoastMode();
    indexer.setIndexerCoastMode();
    shooter.setPivotMotorPercentOutput(0.0);
    Pose2d a = Drivebase.getInstance().getRobotPose();
    System.out.println("Shooter from smart dashboard setting Command End");
    System.out.println("angle: " +shooterAngle.getDegrees());
    System.out.println("Robot thought it was at the position: " + a.getX() + "," + a.getY());
    System.out.println("Command took " + (Timer.getFPGATimestamp()-timeAtStart)+" seconds.");
  }

  @Override
  public boolean isFinished() {
    return !shooter.getShooterIndexerBeambreak2() && !shooter.getShooterIndexerBeambreak1();
  }
}
