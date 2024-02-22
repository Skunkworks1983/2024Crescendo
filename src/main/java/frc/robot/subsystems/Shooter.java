// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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
  public TalonFX shootMotor1;
  public TalonFX shootMotor2;
  public CANSparkMax shooterIndexerMotor;
  Timer timer;
  DigitalInput noteBreak1;
  DigitalInput noteBreak2;


  public enum PivotPosition {
    SHOOTER_TO_STOW, // Default
    SHOOTER_TO_AMP, SHOOTER_TO_SPEAKER
  };

  public PivotPosition pivotPosition = PivotPosition.SHOOTER_TO_STOW;
  // Rotations per minute
  public double flywheelSetpointMPS = 0.0;

  private static Shooter shooter;

  private final DigitalInput pivotMotorForwardLimit =
      new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH);
  private final DigitalInput pivotMotorReverseLimit =
      new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH);

  SmartPIDControllerTalonFX shootingController;
  SmartPIDControllerCANSparkMax indexerController;
  SmartPIDControllerTalonFX pivotController;

  final PositionVoltage positionVoltage = new PositionVoltage(0);
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private Shooter() {
    timer = new Timer();
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor1 = new TalonFX(Constants.IDS.SHOOT_MOTOR1, Constants.CANIVORE_NAME);
    shootMotor2 = new TalonFX(Constants.IDS.SHOOT_MOTOR2, Constants.CANIVORE_NAME);
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    pivotMotor.getConfigurator().apply(talonConfig);
    shootMotor1.getConfigurator().apply(talonConfig);
    shootMotor2.getConfigurator().apply(talonConfig);
    shootMotor2.setControl(new Follower(Constants.IDS.SHOOT_MOTOR1, true));
    shooterIndexerMotor =
        new CANSparkMax(Constants.IDS.SHOOTER_INDEXER_MOTOR, MotorType.kBrushless);
    noteBreak1 = new DigitalInput(Constants.IDS.NOTE_BREAK1);
    noteBreak2 = new DigitalInput(Constants.IDS.NOTE_BREAK2);

    shootingController = new SmartPIDControllerTalonFX(Constants.PIDControllers.ShootingPID.KP,
        Constants.PIDControllers.ShootingPID.KI, Constants.PIDControllers.ShootingPID.KD,
        Constants.PIDControllers.ShootingPID.KF, "Shooter Shoot",
        Constants.PIDControllers.ShootingPID.SMART_PID_ACTIVE, shootMotor1);

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

    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

    if (pivotMotorForwardLimit.get()) {
      pivotMotor.setPosition(Constants.Shooter.SHOOTER_RESTING_POSITION_ROTATIONS);
      pivotMotor.set(0);
    } else if (pivotMotorReverseLimit.get()) {
      pivotMotor.setPosition(Constants.Shooter.SHOOTER_MAX_POSITION_ROTATIONS);
    }

    shootingController.updatePID();
    indexerController.updatePID();
    pivotController.updatePID();
  }

  public void setShooterAngle(Rotation2d desiredRotation, PivotPosition pivotCommand) {
    positionVoltage.Slot = 0;
    pivotMotor.setControl(positionVoltage.withPosition(
        desiredRotation.getDegrees() / Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES));
    this.pivotPosition = pivotCommand;
  }

  public void setShooterAngleVelocity(double radiansPerSecond, PivotPosition pivotCommand) {
    velocityVoltage.Slot = 0;
    pivotMotor.setControl(velocityVoltage
        .withVelocity(Units.radiansToDegrees(radiansPerSecond)
            / Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES)
        .withLimitForwardMotion(pivotMotorForwardLimit.get())
        .withLimitReverseMotion(pivotMotorReverseLimit.get()));
    this.pivotPosition = pivotCommand;
  }

  public void setShooterSpeed(double speedMetersPerSecond) {
    flywheelSetpointMPS = speedMetersPerSecond;
    shootMotor1.setControl(velocityVoltage
        .withVelocity((speedMetersPerSecond * Constants.Shooter.SHOOTER_ROTATIONS_PER_METER)));
  }

  public void setShooterIndexerSpeed(double speedMetersPerSecond) {
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    shooterIndexerMotor.getPIDController()
        .setReference((speedMetersPerSecond * Constants.Shooter.INDEXER_ROTATIONS_PER_METER)
            * Constants.SECONDS_TO_MINUTES, CANSparkMax.ControlType.kVelocity);
  }

  public void setPivotMotorCoastMode() {
    pivotMotor.setControl(new VoltageOut(0));
  }

  public void setShootMotorCoastMode() {
    shootMotor1.setControl(new VoltageOut(0));
  }

  public void setIndexerMotorCoastMode() {
    shooterIndexerMotor.setIdleMode(IdleMode.kCoast);
    shooterIndexerMotor.set(0);
  }

  public boolean getShooterIndexerBeambreak1() {
    return noteBreak1.get();
  }

  public boolean getShooterIndexerBeambreak2() {
    return noteBreak2.get();
  }

  // error in Rot per seconds
  public double getFlywheelError() {
    return shootMotor1.getClosedLoopError().getValue();
  }

  public double getShooterPivotRotation() {
    return pivotMotor.getPosition().getValueAsDouble()
        * Constants.Shooter.PIVOT_MOTOR_ROTATIONS_TO_DEGREES;
  }

  public boolean getLimitSwitchOutput(boolean forwardLimitSwitch) {
    if (forwardLimitSwitch) {
      return pivotMotorForwardLimit.get();
    } else {
      return pivotMotorReverseLimit.get();
    }
  }

  // gets the last run command on the pivot motor
  public PivotPosition getPivotArmCommand() {
    return pivotPosition;
  }

  public boolean canLoadPiece() {
    return getLimitSwitchOutput(false) && (getPivotArmCommand() == PivotPosition.SHOOTER_TO_STOW);
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
