// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class FlywheelPIDTuning extends Command {
  /** Creates a new FlywheelPIDTuning. */
  private Shooter shooter;
  double shooterSetpoint;
  double toleranceTicks;
  double timeAtInit;
  Timer timer;

  public FlywheelPIDTuning() {
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_FLYWHEEL));
    shooterSetpoint = Constants.Shooter.PODIUM_FLYWHEEL_SPEED;
    timer = new Timer();
    toleranceTicks = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(shooterSetpoint);
    timeAtInit = Timer.getFPGATimestamp();
    toleranceTicks = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelMotorCoastMode();
    System.out.println("flywheel tuning command end, time: " + (timer.getFPGATimestamp() - timeAtInit) + " Velocity: " + shooter.shootMotor1.getVelocity().getValueAsDouble() + " Error: " + shooter.getFlywheelError());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getFlywheelError()) < Constants.Shooter.MAX_FLYWHEEL_ERROR) {
      toleranceTicks++;
    }
    else {
      toleranceTicks = 0;
    }
    if(toleranceTicks >= 15)
    {
      return true;
    }
    return false;
  }
}
