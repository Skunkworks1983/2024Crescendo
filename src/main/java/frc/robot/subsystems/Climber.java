// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//This is a stub subsystem
public class Climber extends SubsystemBase {

  TalonFX climbMotor1;
  TalonFX climbMotor2;

  /** Creates a new Climber. */
  public Climber() {
    climbMotor1 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_1, Constants.CANIVORE_NAME);
    climbMotor2 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_2, Constants.CANIVORE_NAME);

  }

  public void setSetpoint(){


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
