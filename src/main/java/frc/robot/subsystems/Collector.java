// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//This is a stub subsystem
public class Collector extends SubsystemBase {

  TalonFX pivotMotor;
  CANSparkMax collectorMotor;

  /** Creates a new Collector. */
  public Collector() {
    collectorMotor = new CANSparkMax(Constants.IDS.COLLECTOR_PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor = new TalonFX(Constants.IDS.COLLECTOR_MOTOR, Constants.CANIVORE_NAME);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void rotateCollector(Rotation2d desiredRotation){


  }

  public void runCollector(double speed){


  }

}
