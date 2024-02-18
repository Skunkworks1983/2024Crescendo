// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class shooterToAmp extends Command {
  
  public Shooter shooter = Shooter.getInstance();

  public shooterToAmp() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d shooterAngle = new Rotation2d(Units.degreesToRadians(Constants.Shooter.SHOOTER_MAX_POSITION_DEGREES) );
    
    shooter.setShooterAngle(shooterAngle, Constants.Shooter.SHOOTER_PIVOT_AMP_COMMAND);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(shooter.getShooterPivotRotation() >= Constants.Shooter.SHOOTER_MAX_POSITION_DEGREES) {
      shooter.setShooterAngleVelocity(0.087, Constants.Shooter.SHOOTER_PIVOT_AMP_COMMAND);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Rotation2d shooterAngle = new Rotation2d();
    shooter.setShooterAngle(shooterAngle, Constants.Shooter.SHOOTER_PIVOT_AMP_COMMAND);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return shooter.getLimitSwitchOutput(true);
  }
}
