// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;
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
  public boolean isFlywheelSpiningWithSetpoint;
  Encoder pivotEncoder;
  double pivotEncoderBaseValue;

  // Meters per second
  public double flywheelSetpointMPS = Constants.Shooter.TEMP_SHOOT_FLYWHEEL_SPEED_RPS
      / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER;

  private static Shooter shooter;

  //private final DigitalInput pivotMotorForwardLimit =
      //new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH);
  //private final DigitalInput pivotMotorReverseLimit =
      //new DigitalInput(Constants.IDS.SHOOTER_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH);

  SmartPIDControllerTalonFX shootingController;
  SmartPIDControllerCANSparkMax indexerController;
  SmartPIDController pivotController;

  final PositionVoltage positionVoltage = new PositionVoltage(0);
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private Shooter() {
    timer = new Timer();
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor1 = new TalonFX(Constants.IDS.SHOOT_MOTOR1);
    shootMotor2 = new TalonFX(Constants.IDS.SHOOT_MOTOR2);

    TalonFXConfiguration talonConfigShootMotor = new TalonFXConfiguration();
    TalonFXConfiguration talonConfigPivotMotor = new TalonFXConfiguration();

    talonConfigShootMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMotor1.getConfigurator().apply(talonConfigShootMotor);
    shootMotor2.getConfigurator().apply(talonConfigShootMotor);
    
    talonConfigPivotMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotMotor.getConfigurator().apply(talonConfigPivotMotor);

    pivotEncoder = new Encoder(3, 4);
    pivotEncoderBaseValue = 0.0;

    shootMotor2.setControl(new Follower(Constants.IDS.SHOOT_MOTOR1, true));
    shooterIndexerMotor =
        new CANSparkMax(Constants.IDS.SHOOTER_INDEXER_MOTOR, MotorType.kBrushless);

    //noteBreak1 = new DigitalInput(Constants.IDS.NOTE_BREAK1);
    //noteBreak2 = new DigitalInput(Constants.IDS.NOTE_BREAK2);

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

    pivotController = new SmartPIDController(Constants.PIDControllers.ShooterPivotPID.KP,
        Constants.PIDControllers.ShooterPivotPID.KI, Constants.PIDControllers.ShooterPivotPID.KD,
        "Shooter Pivot", Constants.PIDControllers.ShooterPivotPID.SMART_PID_ACTIVE);

    // Setting max current on pivot motor for testing.
    pivotMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Shooter.SHOOTER_PIVOT_MAX_AMPS)
            .withSupplyCurrentLimitEnable(true));

    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    isFlywheelSpiningWithSetpoint = false;
  }

  @Override
  public void periodic() {

    if (pivotMotorForwardLimit.get()) {
      pivotEncoder.reset();
      pivotEncoderBaseValue = Constants.Shooter.SHOOTER_RESTING_POSITION_TICKS;
    } else if (pivotMotorReverseLimit.get()) {
      pivotEncoder.reset();
      pivotEncoderBaseValue = Constants.Shooter.SHOOTER_MAX_POSITION_TICKS;
    }


    shootingController.updatePID();
    indexerController.updatePID();
  }

  public void setShooterAngle(Rotation2d desiredRotation) {
    pivotMotor.set(pivotController.calculate(getShooterPivotRotationInDegrees(), desiredRotation.getDegrees()));
  }

  public void setFlywheelSpeed(double speedMetersPerSecond) {
    shootMotor1.setControl(velocityVoltage
        .withVelocity((speedMetersPerSecond * Constants.Shooter.SHOOTER_ROTATIONS_PER_METER)));
    isFlywheelSpiningWithSetpoint = true;
    if (speedMetersPerSecond == 0) {
      isFlywheelSpiningWithSetpoint = false;
    }
  }

  public void setFlywheelSetpoint(double flywheelSpeed) {
    flywheelSetpointMPS = flywheelSpeed;
  }

  public void setShooterIndexerSpeed(double speedMetersPerSecond) {
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    shooterIndexerMotor.getPIDController()
        .setReference((speedMetersPerSecond * Constants.Shooter.INDEXER_ROTATIONS_PER_METER)
            * Constants.SECONDS_TO_MINUTES, CANSparkMax.ControlType.kVelocity);
  }

  public void setFlywheelMotorCoastMode() {
    shootMotor1.setControl(new VoltageOut(0));
    isFlywheelSpiningWithSetpoint = false;
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

  // error in meters per seconds
  public double getFlywheelError() {
    return shootMotor1.getClosedLoopError().getValue()
        / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER;
  }

  public double getShooterPivotRotationInDegrees() {
    return (pivotEncoder.get() + pivotEncoderBaseValue)
        * Constants.Shooter.PIVOT_MOTOR_TICKS_TO_DEGREES;
  }

  public boolean getLimitSwitchOutput(boolean forwardLimitSwitch) {
    /*if (forwardLimitSwitch) {
      return pivotMotorForwardLimit.get();
    } else {
      return pivotMotorReverseLimit.get();
    }*/
    return false;
  }

  // gets the last run command on the pivot motor
  public double getFlywheelSetpoint() {
    return flywheelSetpointMPS;
  }

  public boolean canLoadPiece() {
    return getLimitSwitchOutput(false);
  }

  public void setFlywheelPercentOutput(double percent) {
    shootMotor1.set(percent);
  }

  public void setPivotMotorPercentOutput(double percent) {
    pivotMotor.set(percent);
  }

  public void setIndexerPercentOutput(double percent) {
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    shooterIndexerMotor.set(percent);
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
