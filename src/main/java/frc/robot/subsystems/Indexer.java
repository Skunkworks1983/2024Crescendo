// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.robot.utils.SmartPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerCANSparkMax;

// This is a stub subsystem
public class Indexer extends SubsystemBase {
  private static Indexer indexer;
  CANSparkMax indexerMotor;
  private SmartPIDControllerCANSparkMax indexerMotorController;
  private DigitalInput shooterBeamBreak;

  /** Creates a new Indexer. */
  public Indexer() {
    shooterBeamBreak = new DigitalInput(Constants.IndexerConstants.SHOOTER_BEAM_BREAK);
    indexerMotor = new CANSparkMax(Constants.IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
    indexerMotorController = new SmartPIDControllerCANSparkMax(
        Constants.IndexerConstants.INDEXER_MOTOR_KP, Constants.IndexerConstants.INDEXER_MOTOR_KI,
        Constants.IndexerConstants.INDEXER_MOTOR_KD, Constants.IndexerConstants.INDEXER_MOTOR_KF,
        "IndexerMotor", Constants.IndexerConstants.SET_INDEXER_SMART_PID, indexerMotor);
    indexerMotor.setInverted(true);
  }

  public void setSpeedIndexer(double speedMetersPerSecond) {
    double speedRevolutionsPerSecond =
        ((speedMetersPerSecond / (Math.PI * (Constants.IndexerConstants.INDEXER_WHEEL_DIAMETER))
            * Constants.IndexerConstants.INDEXER_GEAR_RATIO));
    indexerMotor.getPIDController().setReference(speedRevolutionsPerSecond,
        CANSparkBase.ControlType.kVelocity);
  }

  public void stop() {
    indexerMotor.getPIDController().setReference(0, CANSparkBase.ControlType.kVelocity);
  }

  public boolean getBeamBreakSensor() {
    return shooterBeamBreak.get();
  }

  public static Indexer getInstance() {
    if (indexer == null) {
      indexer = new Indexer();
    }
    return indexer;
  }

  public void setPercentOutput(double percent) {
    indexerMotor.set(percent);
  }

  @Override
  public void periodic() {
    indexerMotorController.updatePID();
  }
}
