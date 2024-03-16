// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.untested;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class IntakeShooterFromSource extends Command {
  /** Creates a new IntakePeiceToShooterFromSource. */
  private Shooter shooter;
  private boolean beambreak2Tripped1;
  private boolean beambreak2Tripped2;


  public IntakeShooterFromSource() {
    shooter = Shooter.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_FLYWHEEL), SubsystemGroups.getInstance(Subsystems.ROBOT_INDEXER));
    beambreak2Tripped1 = false;
    beambreak2Tripped2 = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(-1);
    shooter.setIndexerPercentOutput(-Constants.Shooter.SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT);
    System.out.println(
        "Intake Shooter From Source Command Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!beambreak2Tripped1 && shooter.getShooterIndexerBeambreak2()) {
      beambreak2Tripped1 = true;
    }
    if(!beambreak2Tripped2 && beambreak2Tripped1 && !shooter.getShooterIndexerBeambreak2()) {
      shooter.setIndexerPercentOutput(Constants.Shooter.BEAMBREAK1_INDEXER_SPEED);
      shooter.setFlywheelMotorCoastMode();
      beambreak2Tripped2 = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelMotorCoastMode();
    shooter.setShooterIndexerSpeed(0);
    System.out.println(
        "Intake Shooter From Source Command End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return beambreak2Tripped1 && beambreak2Tripped2 && shooter.getShooterIndexerBeambreak2();
  }
}
