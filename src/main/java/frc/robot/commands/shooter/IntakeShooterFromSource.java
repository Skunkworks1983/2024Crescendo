// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class IntakeShooterFromSource extends Command {
  /** Creates a new IntakePeiceToShooterFromSource. */
  private Shooter shooter;
  private boolean beambreak2Tripped;
  Rotation2d shooterAngle;

  public IntakeShooterFromSource() {
    shooter = Shooter.getInstance();
    shooterAngle = new Rotation2d(Units.degreesToRadians(32));
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_FLYWHEEL), SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER), SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(Constants.Shooter.SOURCE_FLYWHEEL_SPEED);
    shooter.setIndexerPercentOutput(-Constants.Shooter.SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT);
    System.out.println(
        "Intake Shooter From Source Command Initialize");
    beambreak2Tripped = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);
    if(shooter.getShooterIndexerBeambreak1() && !shooter.getShooterIndexerBeambreak2() && !beambreak2Tripped) {
      shooter.setIndexerPercentOutput(Constants.Shooter.SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT_SLOW);
      shooter.setFlywheelMotorCoastMode();
      beambreak2Tripped = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelMotorCoastMode();
    shooter.setPivotMotorPercentOutput(0);
    shooter.setShooterIndexerSpeed(0);
    System.out.println(
        "Intake Shooter From Source Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return beambreak2Tripped && shooter.getShooterIndexerBeambreak2();
  }
}
