// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;
import frc.robot.utils.SmartPIDControllerCANSparkMax;
import frc.robot.utils.SmartPIDControllerTalonFX;

//This is a stub subsystem
public class Shooter extends SubsystemBase {

  TalonFX pivotMotor;
  CANSparkMax shootMotor;
  CANSparkMax indexerMotor;
  Timer timer;
  DigitalInput noteBreak;
  private double shootSetPoint;
  private double indexerSetPoint;

  private static Shooter shooter;

  SmartPIDControllerCANSparkMax shootingController;
  SmartPIDControllerCANSparkMax indexerController;
  SmartPIDControllerTalonFX pivotController; 

  final PositionVoltage positionController = new PositionVoltage(0);
  
  private Shooter() {
    timer = new Timer();
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor = new CANSparkMax(Constants.IDS.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.IDS.SHOOTER_INDEXER_MOTOR, MotorType.kBrushless);
    noteBreak = new DigitalInput(Constants.IDS.NOTE_BREAK);

    shootingController = new SmartPIDControllerCANSparkMax(
      Constants.PIDControllers.ShootingPID.KP, 
      Constants.PIDControllers.ShootingPID.KI, 
      Constants.PIDControllers.ShootingPID.KD,
      Constants.PIDControllers.ShootingPID.KF,
      "Shooter Shoot",
      Constants.PIDControllers.ShootingPID.SMART_PID_ACTIVE,
      shootMotor
    );

    indexerController= new SmartPIDControllerCANSparkMax(
      Constants.PIDControllers.ShooterIndexorPID.KP, 
      Constants.PIDControllers.ShooterIndexorPID.KI, 
      Constants.PIDControllers.ShooterIndexorPID.KD,
      Constants.PIDControllers.ShooterIndexorPID.KF,
      "Shooter Indexer", 
      Constants.PIDControllers.ShooterIndexorPID.SMART_PID_ACTIVE,
      indexerMotor
    );
    
    pivotController = new SmartPIDControllerTalonFX(
      Constants.PIDControllers.ShooterPivotPID.KP, 
      Constants.PIDControllers.ShooterPivotPID.KI, 
      Constants.PIDControllers.ShooterPivotPID.KD,
      Constants.PIDControllers.ShooterPivotPID.KF,
      "Shooter Pivot", 
      Constants.PIDControllers.ShooterPivotPID.SMART_PID_ACTIVE, 
      pivotMotor
    );

    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.ForwardLimitAutosetPositionValue = Constants.Shooter.SHOOTER_MAX_POSITION;
    limitSwitchConfigs.ForwardLimitAutosetPositionEnable = true;
    limitSwitchConfigs.ReverseLimitAutosetPositionValue = Constants.Shooter.SHOOTER_RESTING_POSITION;
    limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    
    pivotMotor.getConfigurator().apply(limitSwitchConfigs);
  }

  @Override
  public void periodic() {
    shootMotor.getPIDController().setReference(shootSetPoint, CANSparkMax.ControlType.kVelocity);
    indexerMotor.getPIDController().setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterAngle(Rotation2d desiredRotation) {
    positionController.Slot = 0;
    pivotMotor.setControl(positionController.withPosition(desiredRotation.getDegrees() * Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES));
  }

  public void setShooterSpeed(double speedMetersPerSecond) {
    shootSetPoint = (speedMetersPerSecond / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER) * 60;
  }

  public void setShooterIndexerSpeed(double speedMetersPerSecond) {
    indexerSetPoint = (speedMetersPerSecond / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER) * 60;
  }

  public boolean getShooterIndexerLimitSwitch() {
    return noteBreak.get();
  }

  public double getShooterPivotRotation() {
    return pivotMotor.getPosition().getValueAsDouble() * Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES;
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
