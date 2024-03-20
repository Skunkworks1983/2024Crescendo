// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tuningCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;
import frc.robot.utils.ShooterAimUtils;

public class InterpolationAimShooterCommand extends Command {
  private Shooter shooter;
  Rotation2d shooterAngle;
  double toleranceTicks;
  double timeAtInit;
  Timer timer;
  Drivebase drivebase;
  Indexer indexer;

  public InterpolationAimShooterCommand() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
    timer = new Timer();
    drivebase = Drivebase.getInstance();
  }

  @Override
  public void initialize() {
    Pose2d pose = drivebase.getRobotPose();
    shooterAngle = Rotation2d
        .fromDegrees(ShooterAimUtils.calculateInterpolatedAimAngle(pose.getX(), pose.getY()));
    shooter.setPivotAngleAndSpeed(shooterAngle);

    System.out.println("interpolation aim shooter command init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(shooter.getShooterPivotRotationInDegrees()
        - shooterAngle.getDegrees()) < Constants.Shooter.SHOOTER_PIVOT_TOLARENCE_DEGREES) {
      toleranceTicks++;
    } else {
      toleranceTicks = 0;
    }
    if (toleranceTicks >= Constants.Shooter.SHOOTER_PIVOT_TUNING_SUCCESSFUL_TICKS) {
      //run shooter
    }
   
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("interpolation aim shooter command end");
    shooter.setPivotMotorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
}
