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

public class InterpolationAimShooterWithReloadCommand extends Command {
  private Shooter shooter;
  Rotation2d shooterAngle;
  double toleranceTicks;
  double timeAtInit;
  Timer timer;
  Drivebase drivebase;
  Indexer indexer;

  public InterpolationAimShooterWithReloadCommand() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    drivebase = Drivebase.getInstance();

    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    Pose2d pose = drivebase.getRobotPose();
    shooterAngle = Rotation2d
        .fromDegrees(ShooterAimUtils.calculateInterpolatedAimAngle(pose.getX(), pose.getY()));

    System.out.println("interpolation aim shooter command init");
  }
  
  @Override
  public void execute() {
    if (Math.abs(shooter.getShooterPivotRotationInDegrees()
        - shooterAngle.getDegrees()) < Constants.Shooter.SHOOTER_PIVOT_TOLARENCE_DEGREES) {
      toleranceTicks++;
    } else {
      toleranceTicks = 0;
    }
    if (toleranceTicks >= Constants.Shooter.SHOOTER_PIVOT_TUNING_SUCCESSFUL_TICKS) {
      shooter.setFlywheelSpeed(Constants.Shooter.PODIUM_FLYWHEEL_SPEED);
    }
    shooter.setPivotAngleAndSpeed(shooterAngle);
   
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
    return !shooter.getShooterIndexerBeambreak2() && !shooter.getShooterIndexerBeambreak1() &&
    !indexer.getBeamBreakSensor();
  }
}
