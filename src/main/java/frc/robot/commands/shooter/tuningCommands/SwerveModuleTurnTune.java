// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tuningCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class SwerveModuleTurnTune extends Command {

  private Drivebase drivebase;
  double turnAngle;
  double toleranceTicks;
  double timeAtInit;
  Timer timer;
  boolean passed90Degrees;

  public SwerveModuleTurnTune() {
    drivebase = Drivebase.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
    turnAngle = 90;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeAtInit = 0;
    passed90Degrees = false;
    turnAngle = Constants.DRIVEBASE_TUNING_TURNING_ANGLE_ONE;
    System.out.println("modual tuning command init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(drivebase.getBackRightModuleTurnPos() - Constants.DRIVEBASE_TUNING_TURNING_ANGLE_ONE) > Constants.DRIVEBASE_TUNING_TURNING_TOLERANCE_POWER) {
      passed90Degrees = true;
      timeAtInit = timer.getFPGATimestamp();
      System.out.println("switiching to turning " + (-Constants.DRIVEBASE_TUNING_TURNING_ANGLE_ONE + Constants.DRIVEBASE_TUNING_TURNING_ANGLE_TWO));
    }
    else if (!passed90Degrees) {
      drivebase.setBackRightModuleTurnPos(Constants.DRIVEBASE_TUNING_TURNING_ANGLE_ONE);
    }

    if(passed90Degrees) {
      drivebase.setBackRightModuleTurnPos(Constants.DRIVEBASE_TUNING_TURNING_ANGLE_TWO);
    }

    SmartDashboard.putNumber("frontRight turn pos", drivebase.getBackRightModuleTurnPos());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("modual tuning command end, time: " + (timer.getFPGATimestamp() - timeAtInit));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(drivebase.getBackRightModuleTurnError()) < Constants.DRIVEBASE_TUNING_TURNING_TOLERANCE) {
      toleranceTicks++;
    } else {
      toleranceTicks = 0;
    }
    if (toleranceTicks >= Constants.DRIVEBASE_TUNING_TICK_COUNT) {
      return true;
    }
    return false;
  }
}
