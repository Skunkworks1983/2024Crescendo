// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SetFieldTarget;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

public class AimNoteToPass extends Command {
    private Shooter shooter;
  Drivebase drivebase;
  /** Creates a new PassNote. */
  public AimNoteToPass() {
    shooter = Shooter.getInstance();
    drivebase = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double shootDistance = drivebase.getRobotPose().getTranslation().getDistance(
      Constants.Targeting.FieldTarget.PASS_TARGET.get().get().toTranslation2d()
    );
    double hSpeed = Constants.Shooter.PASS_HORIZONTAL_SPEED;
    double passTime = shootDistance/hSpeed;
    double ySpeed = Math.pow(passTime,2) * .5 * Constants.ACCELERATION_DUE_TO_GRAVITY;
    Rotation2d pivotRotation = new Rotation2d((Math.PI/2.0)-Math.atan2(ySpeed,hSpeed));
    shooter.setPivotAngleAndSpeed(pivotRotation);
    double flywheelSpeed = Math.sqrt(Math.pow(hSpeed,2)+Math.pow(passTime,2));
    shooter.setFlywheelSetpoint(flywheelSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
