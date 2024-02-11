// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;

//This is a stub subsystem
public class Shooter extends SubsystemBase {

  TalonFX pivotMotor;
  CANSparkMax shootMotor;
  CANSparkMax indexerMotor;
  Timer timer;
  DigitalInput noteBreak;

  private static Shooter shooter;

  SmartPIDController shootingController = new SmartPIDController(
    Constants.PIDControllers.ShootingPID.KP, 
    Constants.PIDControllers.ShootingPID.KI, 
    Constants.PIDControllers.ShootingPID.KD,
    "Shooting",
    Constants.PIDControllers.ShootingPID.SMART_PID_ACTIVE
  );
  
  private Shooter() {
    timer = new Timer();
    pivotMotor = new TalonFX(Constants.IDS.SHOOTER_PIVOT_MOTOR, Constants.CANIVORE_NAME);
    shootMotor = new CANSparkMax(Constants.IDS.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.IDS.SHOOTER_INDEXER_MOTOR, MotorType.kBrushless);
    noteBreak = new DigitalInput(Constants.IDS.NOTE_BREAK);

    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.ForwardLimitAutosetPositionValue = Constants.Shooter.SHOOTER_MAX_POSITION;
    limitSwitchConfigs.ForwardLimitAutosetPositionEnable = true;
    limitSwitchConfigs.ReverseLimitAutosetPositionValue = Constants.Shooter.SHOOTER_RESTING_POSITION;
    limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    
    pivotMotor.getConfigurator().apply(limitSwitchConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterRotation(Rotation2d desiredRotation) {
    
  }

  public void setShooterSpeed(double speedMetersPerSecond) {

    shootMotor.set(
      shootingController.calculate(
        (shootMotor.getEncoder().getVelocity() * Constants.Shooter.ROTATIONS_PER_METER)/60, 
        speedMetersPerSecond
      )
    );
  }

  public boolean setShooterIndexerSpeed(double speed) {
    if(!noteBreak.get()) {
      indexerMotor.set(speed);
    }
    else{
      indexerMotor.set(0);
    }

    return noteBreak.get();
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }
}
