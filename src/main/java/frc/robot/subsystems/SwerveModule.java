// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SwerveModule extends SubsystemBase {

  TalonFX driveMotor;
  TalonFX turnMotor;
  CANcoder turnEncoder;
  String canivoreName;

  PIDController turnController = new PIDController(
    Constants.PIDControllers.TurnPID.KP, 
    Constants.PIDControllers.TurnPID.KI, 
    Constants.PIDControllers.TurnPID.KD
  );

  final VelocityVoltage velocityController = new VelocityVoltage(0);

  public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId, double turnEncoderOffset, String canivoreName) {
    driveMotor = new TalonFX(driveMotorId, canivoreName);
    turnMotor = new TalonFX(turnMotorId, canivoreName);
    turnEncoder = new CANcoder(turnEncoderId, canivoreName);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = -turnEncoderOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(canCoderConfig);
    turnController.enableContinuousInput(-180, 180); // Pid controller will loop from -180 to 180 continuously
    turnController.setTolerance(Constants.PIDControllers.TurnPID.TURN_PID_TOLERANCE); // sets the tolerance of the turning pid controller.

    //reseting the configuration to default
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(talonConfig);
    turnMotor.getConfigurator().apply(talonConfig);
    velocityController.Slot = 0;
    Slot0Configs slot0Configs = new Slot0Configs();
  
    slot0Configs.kP = Constants.PIDControllers.DrivePID.KP;
    slot0Configs.kI = Constants.PIDControllers.DrivePID.KI;
    slot0Configs.kD = Constants.PIDControllers.DrivePID.KD;
    slot0Configs.kV = Constants.PIDControllers.DrivePID.KF;

    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
    driveConfigs.Slot0 = slot0Configs;
    driveConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    driveMotor.getConfigurator().apply(driveConfigs);
  }

  // sets drive motor in velocity mode (set feet per second)
  public void setDriveMotorVelocity(double feetPerSecond) {

    double revsPerSecond = feetPerSecond * Constants.DrivebaseInfo.REVS_PER_FOOT;
    // SmartDashboard.putNumber("velocity mode ticks speed", speed);
    // SmartDashboard.putNumber("Velocity error", driveMotor.getClosedLoopError());
    velocityController.Slot = 0;
    driveMotor.setControl(velocityController.withVelocity(revsPerSecond));
  }

  public void setTurnMotorSpeed(double speed) {
    turnMotor.set(speed);
  }

  // gets drive encoder as distance traveled in feet
  public double getDriveEncoderPosition() {

    double distance = driveMotor.getPosition().getValue() / Constants.DrivebaseInfo.REVS_PER_FOOT;
    SmartDashboard.putNumber("drive encoder " + turnMotor, distance);
    return distance;
  }

  // returns drive encoder velocity in feet per second
  public double getDriveEncoderVelocity() {

    double feetPerSecond = driveMotor.getVelocity().getValue() / Constants.DrivebaseInfo.REVS_PER_FOOT;
    SmartDashboard.putNumber("drive encoder velocity", feetPerSecond);
    return feetPerSecond;
  }

  /**gets turn encoder as degrees, -180 180*/ 
  public double getTurnEncoder() {               
    // multiplying absolute postion by 360 to convert from +- .5 to +- 180
    // gets the absoulte position of the encoder. getPosition() returns relative position.
    double angle = turnEncoder.getAbsolutePosition().getValue()*360;   
    SmartDashboard.putNumber("turn encoder" + turnMotor.getDeviceID(), angle);

    return angle;
  }


  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Units.feetToMeters(getDriveEncoderPosition()), Rotation2d.fromDegrees(getTurnEncoder()));
  }


  public void setState(SwerveModuleState desiredState) {
    SwerveModuleState optimized = SwerveModuleState.optimize(
      desiredState, 
      new Rotation2d(Units.degreesToRadians(getTurnEncoder())));

    setDriveMotorVelocity(Units.metersToFeet(optimized.speedMetersPerSecond));
    turnController.setSetpoint(optimized.angle.getDegrees()); // set setpoint
    
    double speed = -turnController.calculate(getTurnEncoder()); // calculate speed
    boolean atSetpoint = turnController.atSetpoint();

    SmartDashboard.putNumber("turn pid error", turnController.getPositionError());
    SmartDashboard.putNumber("setting turn speed",
        MathUtil.clamp(speed, Constants.PIDControllers.TurnPID.PID_LOW_LIMIT, Constants.PIDControllers.TurnPID.PID_HIGH_LIMIT));

    if (!atSetpoint) {
      // clamp and set speed
      setTurnMotorSpeed(MathUtil.clamp(speed, Constants.PIDControllers.TurnPID.PID_LOW_LIMIT, Constants.PIDControllers.TurnPID.PID_HIGH_LIMIT));
    }
  }
}