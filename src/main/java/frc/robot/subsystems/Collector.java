// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerCANSparkMax;
import frc.robot.utils.SmartPIDControllerTalonFX;

// This is a stub subsystem
public class Collector extends SubsystemBase {

  TalonFX rightPivotMotor;
  TalonFX leftPivotMotor;
  CANSparkMax topIntakeMotor;
  CANSparkMax bottomIntakeMotor;

  private static SmartPIDControllerCANSparkMax topIntakeMotorSpeedController;
  private static SmartPIDControllerTalonFX pivotMotorController;
  private static Collector collector;

  final PositionVoltage positionVoltage = new PositionVoltage(0);
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0);


  /** Creates a new Collector. */
  private Collector() {
    topIntakeMotor = new CANSparkMax(Constants.Collector.TOP_INTAKE_MOTOR, MotorType.kBrushless);
    bottomIntakeMotor =
        new CANSparkMax(Constants.Collector.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
    rightPivotMotor = new TalonFX(Constants.Collector.RIGHT_PIVOT_MOTOR);
    leftPivotMotor = new TalonFX(Constants.Collector.LEFT_PIVOT_MOTOR);

    bottomIntakeMotor.follow(topIntakeMotor);
    leftPivotMotor.setControl(new Follower(Constants.Collector.RIGHT_PIVOT_MOTOR, true));

    topIntakeMotor.getEncoder().setVelocityConversionFactor(Constants.Collector.INTAKE_GEAR_RATIO
        / (Constants.Collector.INTAKE_ROLLER_DIAMETER * Math.PI));
    topIntakeMotorSpeedController =
        new SmartPIDControllerCANSparkMax(Constants.PIDControllers.TopCollectorIntakePID.KP,
            Constants.PIDControllers.TopCollectorIntakePID.KI,
            Constants.PIDControllers.TopCollectorIntakePID.KD,
            Constants.PIDControllers.TopCollectorIntakePID.FF, "intake motor",
            Constants.PIDControllers.TopCollectorIntakePID.SMART_PID_ACTIVE, topIntakeMotor);

    pivotMotorController =
        new SmartPIDControllerTalonFX(Constants.PIDControllers.CollectorPivotPID.KP,
            Constants.PIDControllers.CollectorPivotPID.KI,
            Constants.PIDControllers.CollectorPivotPID.KD,
            Constants.PIDControllers.CollectorPivotPID.FF, "pivot motor controller",
            Constants.PIDControllers.CollectorPivotPID.SMART_PID_ACTIVE, rightPivotMotor);

    // Setting voltage limit on the collector pivot for testing.
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightPivotMotor.getConfigurator().apply(config);
    leftPivotMotor.getConfigurator().apply(config);

    rightPivotMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Collector.COLLECTOR_PIVOT_MAX_AMPS)
            .withSupplyCurrentLimitEnable(true));
  }

  // meters per second
  public void intakeNotes(double metersPerSecond) {
    topIntakeMotor.getPIDController()
        .setReference(((metersPerSecond / (Math.PI * Constants.Collector.INTAKE_ROLLER_DIAMETER))
            * Constants.Collector.INTAKE_GEAR_RATIO), CANSparkMax.ControlType.kVelocity);
    topIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public boolean isStowed() {
    return (Math.abs(Constants.Collector.COLLECTOR_STOW_POS
        - rightPivotMotor.getPosition().getValue()) < Constants.Collector.COLLECTOR_POS_TOLERANCE);
  }

  public boolean isAtFloor() {
    return (Math.abs(Constants.Collector.COLLECTOR_FLOOR_POS
        - rightPivotMotor.getPosition().getValue()) < Constants.Collector.COLLECTOR_POS_TOLERANCE);
  }

  public void setCollectorPos(double angle) {
    rightPivotMotor.setControl(positionVoltage.withPosition(angle * Constants.Collector.DEGREES_TO_PIVOT_MOTOR_ROTATIONS));
  }

  public void setIntakeCoastMode() {
    topIntakeMotor.setIdleMode(IdleMode.kCoast);
    topIntakeMotor.set(0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCollectorPivotVelocity(double speed) {
    rightPivotMotor.setControl(velocityVoltage.withVelocity(speed));
  }

  public void setPercentOutput(double percent) {
    topIntakeMotor.set(percent);
    bottomIntakeMotor.set(percent);
  }

  public double getCollectorPos() {
    return rightPivotMotor.getPosition().getValueAsDouble() / Constants.Collector.DEGREES_TO_PIVOT_MOTOR_ROTATIONS;
  }


  public static Collector getInstance() {
    if (collector == null) {
      collector = new Collector();
    }
    return collector;
  }
}
