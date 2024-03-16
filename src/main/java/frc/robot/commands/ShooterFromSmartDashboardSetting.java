// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterFromSmartDashboardSetting extends Command {

  private Shooter shooter;
  boolean isTurning;
  Rotation2d shooterAngle;

  public ShooterFromSmartDashboardSetting() {
    SmartDashboard.putNumber( "flywheelSpeed", 0.0);
    SmartDashboard.putNumber("shooterAngle",0.0);
    shooter = Shooter.getInstance();
    isTurning = false;
    shooterAngle = new Rotation2d(SmartDashboard.getNumber("shooterAngle",0.0));
    addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
  }

  @Override
  public void initialize() {
    shooter.setFlywheelSetpoint(SmartDashboard.getNumber("flywheelSpeed",0.0));
    System.out.println(
        "Shooter from smart dashboard setting Command Initialize");
  }

  @Override
  public void execute() {
    shooter.setPivotAngleAndSpeed(shooterAngle);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelSetpoint(Constants.Shooter.PODIUM_FLYWHEEL_SPEED);
    shooter.setPivotMotorPercentOutput(0);
    System.out.println(
        "Shooter from smart dashboard setting Command End");
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
