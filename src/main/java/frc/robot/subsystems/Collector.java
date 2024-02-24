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
import frc.robot.utils.SmartPIDControllerCANSparkMax;

// This is a stub subsystem
public class Collector extends SubsystemBase {


  CANSparkMax pivotMotor;
  // TalonFX pivotMotor;
  CANSparkMax intakeMotor;

  private static SmartPIDControllerCANSparkMax intakeMotorSpeedController;
  private static SmartPIDControllerCANSparkMax pivotMotorController;
  private static Collector collector;

  private double pivotSetPoint;
  private double speedSetPoint;

  /** Creates a new Collector. */
  private Collector() {
    intakeMotor = new CANSparkMax(Constants.Collector.COLLECTOR_PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(Constants.Collector.COLLECTOR_MOTOR, MotorType.kBrushless);

    intakeMotor.getEncoder().setVelocityConversionFactor(Constants.Collector.INTAKE_GEAR_RATIO
        / (Constants.Collector.INTAKE_ROLLER_DIAMETER * Math.PI));
    intakeMotorSpeedController =
        new SmartPIDControllerCANSparkMax(Constants.PIDControllers.CollectorIntakePID.KP,
            Constants.PIDControllers.CollectorIntakePID.KI,
            Constants.PIDControllers.CollectorIntakePID.KD,
            Constants.PIDControllers.CollectorIntakePID.FF, "intake motor speed controller",
            Constants.PIDControllers.CollectorIntakePID.SMART_PID_ACTIVE, intakeMotor);

    pivotMotorController =
        new SmartPIDControllerCANSparkMax(Constants.PIDControllers.CollectorPivotPID.KP,
            Constants.PIDControllers.CollectorPivotPID.KI,
            Constants.PIDControllers.CollectorPivotPID.KD,
            Constants.PIDControllers.CollectorPivotPID.FF, "pivot motor controller",
            Constants.PIDControllers.CollectorPivotPID.SMART_PID_ACTIVE, pivotMotor);

    // Setting voltage limit on the collector pivot for testing.
    pivotMotor.setSmartCurrentLimit(Constants.Collector.COLLECTOR_PIVOT_MAX_AMPS);
  }

  public void intakeNotes(double setPoint) {
    speedSetPoint = ((setPoint / (Math.PI * Constants.Collector.INTAKE_ROLLER_DIAMETER))// wheel
                                                                                        // rotion
        * Constants.Collector.INTAKE_GEAR_RATIO);
  }

  public boolean isStowed() {
    return (Math.abs(Constants.Collector.COLLECTOR_STOW_POS
        - pivotMotor.getEncoder().getPosition()) < Constants.Collector.COLLECTOR_POS_TOLERANCE);
  }

  public boolean isAtFloor() {
    return (Math.abs(Constants.Collector.COLLECTOR_FLOOR_POS
        - pivotMotor.getEncoder().getPosition()) < Constants.Collector.COLLECTOR_POS_TOLERANCE);
  }

  public void setCollectorPos(double angle) {
    pivotSetPoint = angle;
  }

  public void periodic() {

    pivotMotor.getPIDController().setReference(pivotSetPoint, CANSparkMax.ControlType.kPosition);
    intakeMotor.getPIDController().setReference(speedSetPoint, CANSparkMax.ControlType.kVelocity);
    // This method will be called once per scheduler run
  }

  public static Collector getInstance() {
    if (collector == null) {
      collector = new Collector();
    }
    return collector;
  }
}
