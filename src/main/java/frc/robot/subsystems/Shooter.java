// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double lastFlywheelSpeed;

  public enum LimitSwitch {
    FORWARD_LIMIT_SWITCH, REVERSE_LIMIT_SWITCH
  }

  // Meters per second
  public double flywheelSetpointMPS = Constants.Shooter.STOW_FLYWHEEL_SPEED;

  private static Shooter shooter;

  SmartPIDControllerTalonFX shootingController;
  SmartPIDControllerCANSparkMax indexerController;
  SmartPIDController pivotController;
  double pivotKf = 0.0;
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
    talonConfigPivotMotor.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
    talonConfigPivotMotor.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonConfigPivotMotor.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonConfigPivotMotor.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
    pivotMotor.getConfigurator().apply(talonConfigPivotMotor);

    pivotEncoder = new Encoder(Constants.IDS.SHOOTER_PIVOT_ENCODER_PIN_1,
        Constants.IDS.SHOOTER_PIVOT_ENCODER_PIN_2);
    pivotEncoderBaseValue = Constants.Shooter.SHOOTER_RESTING_POSITION_TICKS;

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

    pivotController = new SmartPIDController(Constants.PIDControllers.ShooterPivotPID.KP,
        Constants.PIDControllers.ShooterPivotPID.KI, Constants.PIDControllers.ShooterPivotPID.KD,
        "Shooter Pivot", Constants.PIDControllers.ShooterPivotPID.SMART_PID_ACTIVE);

    // Setting max current on pivot motor for testing.
    pivotMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Shooter.SHOOTER_PIVOT_MAX_AMPS)
            .withSupplyCurrentLimitEnable(true));

    pivotController.setTolerance(Constants.Shooter.SHOOTER_PIVOT_PID_TOLERANCE);
    pivotController.setSetpoint(Constants.Shooter.SHOOTER_RESTING_POSITION.getDegrees());
    pivotController.calculate(getShooterPivotRotationInDegrees());
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    isFlywheelSpiningWithSetpoint = false;
  }

  @Override
  public void periodic() {

    if (getLimitSwitchOutput(LimitSwitch.FORWARD_LIMIT_SWITCH)) {
      pivotEncoder.reset();
      pivotEncoderBaseValue = Constants.Shooter.SHOOTER_MAX_POSITION_TICKS;
    } else if (getLimitSwitchOutput(LimitSwitch.REVERSE_LIMIT_SWITCH)) {
      pivotEncoder.reset();
      pivotEncoderBaseValue = Constants.Shooter.SHOOTER_RESTING_POSITION_TICKS;
    }

    shootingController.updatePID();
    indexerController.updatePID();
    // SmartDashboard.putNumber("Shooter Shoot Velocity",
    // shootMotor1.getVelocity().getValueAsDouble());
   // SmartDashboard.putNumber("Shooter Shoot Error", getFlywheelError());

    pivotKf = 0.0375 * Math.sin(Units.degreesToRadians(getShooterPivotRotationInDegrees()));
  }

  // needs to be run in execute
  public void setPivotAngleAndSpeed(Rotation2d desiredRotation) {
    double controllerCalculation =
        pivotController.calculate(getShooterPivotRotationInDegrees(), desiredRotation.getDegrees())
            + pivotKf;

    pivotMotor.setControl(new DutyCycleOut(controllerCalculation));
    SmartDashboard.putNumber("Shooter Pivot Motor Output", controllerCalculation);
  }

  /** Returns true if the shooter pivot PID controller is at it's setpoint */
  public boolean isPivotAtSetpoint() {
    return pivotController.atSetpoint();
  }

  public void setFlywheelSpeed(double speedMetersPerSecond) {
    if (speedMetersPerSecond != lastFlywheelSpeed) {
      shootMotor1.setControl(velocityVoltage
          .withVelocity((speedMetersPerSecond * Constants.Shooter.SHOOTER_ROTATIONS_PER_METER)));
    }
    lastFlywheelSpeed = speedMetersPerSecond;
    isFlywheelSpiningWithSetpoint = true;
    if (speedMetersPerSecond == 0) {
      isFlywheelSpiningWithSetpoint = false;
    }
  }

  public double getShooterPivotError() {
    return pivotController.getPositionError();
  }
  public double getPivotSetPoint(){
    return pivotController.getSetpoint();
  } 
  public void setFlywheelSetpoint(double flywheelSpeed) {
    flywheelSetpointMPS = flywheelSpeed;
  }

  public double getFlywheelVelocity() {
    return shootMotor1.getVelocity().getValueAsDouble()
        / Constants.Shooter.SHOOTER_ROTATIONS_PER_METER;
  }

  public double getFlywheelError() {
    return getFlywheelVelocity() - flywheelSetpointMPS;
  }

  public void setShooterIndexerSpeed(double speedMetersPerSecond) {
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    shooterIndexerMotor.getPIDController()
        .setReference((speedMetersPerSecond * Constants.Shooter.INDEXER_ROTATIONS_PER_METER)
            * Constants.SECONDS_TO_MINUTES, CANSparkMax.ControlType.kVelocity);
  }

  public void setFlywheelMotorCoastMode() {
    shootMotor1.setControl(new VoltageOut(0));
    lastFlywheelSpeed = 0;
    isFlywheelSpiningWithSetpoint = false;
  }

  public void setIndexerMotorCoastMode() {
    shooterIndexerMotor.setIdleMode(IdleMode.kCoast);
    shooterIndexerMotor.set(0);
  }

  public boolean getShooterIndexerBeambreak1() {
    return !noteBreak1.get();
  }

  public boolean getShooterIndexerBeambreak2() {
    return !noteBreak2.get();
  }

  public double getShooterPivotRotationInDegrees() {
    return (pivotEncoder.get() + pivotEncoderBaseValue)
        * Constants.Shooter.PIVOT_MOTOR_TICKS_TO_DEGREES;
  }

  public boolean getLimitSwitchOutput(LimitSwitch limitSwitch) {
    if (limitSwitch == LimitSwitch.FORWARD_LIMIT_SWITCH) {
      return pivotMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    } else if (limitSwitch == LimitSwitch.REVERSE_LIMIT_SWITCH) {
      return pivotMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }
    return false;
  }

  // gets the last run command on the pivot motor
  public double getFlywheelSetpoint() {
    return flywheelSetpointMPS;
  }

  public boolean canLoadPiece() {
    return getLimitSwitchOutput(LimitSwitch.REVERSE_LIMIT_SWITCH);
  }

  public void setFlywheelPercentOutput(double percent) {
    shootMotor1.set(percent);
    lastFlywheelSpeed = percent;
  }

  public void setPivotMotorPercentOutput(double percent) {
    pivotMotor.set(percent);
  }

  public void setIndexerPercentOutput(double percent) {
    shooterIndexerMotor.setIdleMode(IdleMode.kBrake);
    shooterIndexerMotor.set(percent);
  }

  public double getShooterSetpoint() {
    return pivotController.getSetpoint();
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
