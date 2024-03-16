// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tuningCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class shooterPivotTuneCommand extends Command {
  /** Creates a new shooterPivotTuneCommand. */
  private Shooter shooter;
  Rotation2d shooterAngle;
  double toleranceTicks;
  double timeAtInit;
  Timer timer;

  public shooterPivotTuneCommand() {
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
    shooterAngle =
        new Rotation2d(Units.degreesToRadians(Constants.Shooter.SHOOTER_PIVOT_TESTING_ANGLE));
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("pivot tuning command init");
    timeAtInit = timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("pivot tuning command end, time: " + (timer.getFPGATimestamp() - timeAtInit));
    shooter.setPivotMotorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getShooterPivotRotationInDegrees()
        - Constants.Shooter.SHOOTER_PIVOT_TESTING_ANGLE) < Constants.Shooter.SHOOTER_PIVOT_TOLARENCE_DEGREES) {
      toleranceTicks++;
    } else {
      toleranceTicks = 0;
    }
    if (toleranceTicks >= Constants.Shooter.SHOOTER_PIVOT_TUNING_SUCCESSFUL_TICKS) {
      return true;
    }
    return false;
  }
}
