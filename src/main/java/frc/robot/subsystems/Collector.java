// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;

//This is a stub subsystem
public class Collector extends SubsystemBase 
{


  CANSparkMax pivotMotor;
  //TalonFX pivotMotor;
  CANSparkMax intakeMotor;

  private static SmartPIDController intakeMotorSpeedController;
  private static Collector collector;

  /** Creates a new Collector. */
  public Collector() 
  {
    intakeMotor = new CANSparkMax(Constants.Collector.COLLECTOR_PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(Constants.Collector.COLLECTOR_MOTOR, MotorType.kBrushless);

    intakeMotor.getEncoder().setVelocityConversionFactor(Constants.Collector.INTAKE_GEAR_RATIO / (Constants.Collector.INTAKE_ROLLER_DIAMETER * Math.PI));
  intakeMotorSpeedController = new SmartPIDController
   (
    Constants.PIDControllers.CollectorPID.KP,
     Constants.PIDControllers.CollectorPID.KI,
     Constants.PIDControllers.CollectorPID.KD,
    "intake motor",
    false
   );
  }

  public void intakeNotes(double setPoint)
  {
    intakeMotor.set(((setPoint / (Math.PI * Constants.Collector.INTAKE_ROLLER_DIAMETER))// wheel rotion
     * Constants.Collector.INTAKE_GEAR_RATIO));
    double currentSpeed = intakeMotor.getEncoder().getVelocity();
    intakeMotor.set(intakeMotorSpeedController.calculate(currentSpeed,setPoint));
  }

  public void setCollectorPos(double angle)
  {
    pivotMotor.set(angle/360.0 * Constants.Collector.PIVOT_GEAR_RATIO);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  
  public void rotateCollector(Rotation2d desiredRotation) 
  {

  }

  public void runCollector(double speed) 
  {

  }

  public static Collector getInstance() 
  {
    if (collector == null) {
      collector = new Collector();
    }
    return collector;
  }
}
