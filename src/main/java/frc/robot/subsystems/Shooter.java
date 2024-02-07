// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//This is a stub subsystem
public class Shooter extends SubsystemBase {

  TalonFX pivotMotor;
  TalonFX shootMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor = new TalonFX(Constants.IDS.SHOOTER_MOTOR, Constants.CANIVORE_NAME);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterRotation(Rotation2d desiredRotation) {}

  public void runShooter(double speed) {}
}
