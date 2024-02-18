// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.utils.SmartPIDControllerCANSparkMax;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Shooter extends SubsystemBase {

  public TalonFX pivotMotor;
  public CANSparkMax shootMotor;
  public CANSparkMax shooterIndexerMotor;
  Timer timer;
  DigitalInput noteBreak;
  public String pivotCommand = Constants.Shooter.SHOOTER_PIVOT_STOW_COMMAND;

  private static Shooter shooter;

  private final DigitalInput pivotMotorForwardLimit = new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH);
  private final DigitalInput pivotMotorReverseLimit = new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH);

  SmartPIDControllerCANSparkMax shootingController;
  SmartPIDControllerCANSparkMax indexerController;
  SmartPIDControllerTalonFX pivotController;

  final PositionVoltage positionController = new PositionVoltage(0);
  final VelocityVoltage velocityController = new VelocityVoltage(0);

  private Shooter() {
    timer = new Timer();
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor = new CANSparkMax(Constants.IDS.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    shooterIndexerMotor = new CANSparkMax(Constants.IDS.SHOOTER_INDEXER_MOTOR, MotorType.kBrushless);
    noteBreak = new DigitalInput(Constants.IDS.NOTE_BREAK);

    shootingController = new SmartPIDControllerCANSparkMax(Constants.PIDControllers.ShootingPID.KP,
        Constants.PIDControllers.ShootingPID.KI, Constants.PIDControllers.ShootingPID.KD,
        Constants.PIDControllers.ShootingPID.KF, "Shooter Shoot",
        Constants.PIDControllers.ShootingPID.SMART_PID_ACTIVE, shootMotor);

    indexerController =
        new SmartPIDControllerCANSparkMax(Constants.PIDControllers.ShooterIndexerPID.KP,
            Constants.PIDControllers.ShooterIndexerPID.KI,
            Constants.PIDControllers.ShooterIndexerPID.KD,
            Constants.PIDControllers.ShooterIndexerPID.KF, "Shooter Indexer",
            Constants.PIDControllers.ShooterIndexerPID.SMART_PID_ACTIVE, shooterIndexerMotor);

    pivotController = new SmartPIDControllerTalonFX(Constants.PIDControllers.ShooterPivotPID.KP,
        Constants.PIDControllers.ShooterPivotPID.KI, Constants.PIDControllers.ShooterPivotPID.KD,
        Constants.PIDControllers.ShooterPivotPID.KF, "Shooter Pivot",
        Constants.PIDControllers.ShooterPivotPID.SMART_PID_ACTIVE, pivotMotor);
  }

  @Override
  public void periodic() {

    if(pivotMotorForwardLimit.get()) {
      pivotMotor.setPosition(Constants.Shooter.SHOOTER_RESTING_POSITION_ROTATIONS);
    } else if(pivotMotorReverseLimit.get()) {
      pivotMotor.setPosition(Constants.Shooter.SHOOTER_MAX_POSITION_ROTATIONS);
    }
    
    shootingController.updatePID();
    indexerController.updatePID();
    pivotController.updatePID();
  }

  public void setShooterAngle(Rotation2d desiredRotation, String pivotCommand) {
    positionController.Slot = 0;
    pivotMotor.setControl(positionController.withPosition(
        desiredRotation.getDegrees()/Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES)
        .withLimitForwardMotion(pivotMotorForwardLimit.get())
        .withLimitReverseMotion(pivotMotorReverseLimit.get()));
    this.pivotCommand = pivotCommand;
  }

  public void setShooterAngleVelocity(double radiansPerSecond, String pivotCommand) {
    velocityController.Slot = 0;
    pivotMotor.setControl(velocityController.withVelocity(Units.radiansToDegrees(radiansPerSecond)/Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES)
    .withLimitForwardMotion(pivotMotorForwardLimit.get())
    .withLimitReverseMotion(pivotMotorReverseLimit.get()));
    this.pivotCommand = pivotCommand;
  }

  public void setShooterSpeed(double speedMetersPerSecond) {
    shootMotor.getPIDController().setReference((speedMetersPerSecond / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER) * 60, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterIndexerSpeed(double speedMetersPerSecond) {
    shooterIndexerMotor.getPIDController().setReference((speedMetersPerSecond / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER) * 60, CANSparkMax.ControlType.kVelocity);
  }

  public boolean getShooterIndexerLimitSwitch() {
    return noteBreak.get();
  }

  public double getShooterPivotRotation() {
    return pivotMotor.getPosition().getValueAsDouble()
        * Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES;
  }

  public boolean getLimitSwitchOutput(boolean forwardLimitSwitch) {
    if(forwardLimitSwitch) {
      return pivotMotorForwardLimit.get();
    }
    else {
      return pivotMotorReverseLimit.get();
    }
  }

  public String getPivotArmCommand() {
    return pivotCommand;
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
