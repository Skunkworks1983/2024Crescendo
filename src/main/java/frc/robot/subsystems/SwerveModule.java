// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
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
  CANSparkMax turnMotor;
  CANcoder turnEncoder;

  PIDController turnController = new PIDController(Constants.PIDControllers.TurnPID.KP, Constants.PIDControllers.TurnPID.KI, Constants.PIDControllers.TurnPID.KD);
  final VelocityVoltage velocityController = new VelocityVoltage(0);


  // Constructor
  public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId, double turnEncoderOffset) {

    driveMotor = new TalonFX(driveMotorId, "Canivore_1");
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    turnEncoder = new CANcoder(turnEncoderId, "Canivore_1");

    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
    CANcoderConfig.MagnetSensor.MagnetOffset = -turnEncoderOffset;
    CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(CANcoderConfig);

    turnController.enableContinuousInput(-180, 180); // Pid controller will loop from -180 to 180 continuously
    turnController.setTolerance(2); // sets the tolerance of the turning pid controller.


    var slot0Configs = new Slot0Configs();
  
    slot0Configs.kP = .1/204.8;
    slot0Configs.kD = 0;
    slot0Configs.kV = 0;
    slot0Configs.kI = 0;
    driveMotor.getConfigurator().apply(slot0Configs);
  }


  public void setDriveMotorVelocity(double feetPerSecond) { // sets drive motor in velocity mode (set feet per second)

    double revsPerSecond = feetPerSecond * Constants.DrivebaseInfo.REVS_PER_FOOT;
    // SmartDashboard.putNumber("velocity mode ticks speed", speed);
    // SmartDashboard.putNumber("Velocity error", driveMotor.getClosedLoopError());
    velocityController.Slot = 0;
    driveMotor.setControl(velocityController.withVelocity(revsPerSecond));
  }

  public void setTurnMotor(double speed) {

    turnMotor.set(speed);
  }

  public double getDriveEncoderPosition() {             // gets drive encoder as distance traveled in feet

    double distance = driveMotor.getPosition().getValue() / Constants.DrivebaseInfo.REVS_PER_FOOT;
    SmartDashboard.putNumber("drive encoder", distance);
    return distance;
  }

  public double getDriveEncoderVelocity() {             // returns drive encoder velocity in feet per second

    double feetPerSecond = driveMotor.getVelocity().getValue() / Constants.DrivebaseInfo.REVS_PER_FOOT;
    SmartDashboard.putNumber("drive encoder velocity", feetPerSecond);
    return feetPerSecond;
  }

  /**gets turn encoder as degrees, -180 180*/ 
  public double getTurnEncoder() {   
                       
    // multiplying absolute postion by 360 to convert from +- .5 to +- 180
    // gets the absoulte position of the encoder. getPosition() returns relative position.
    double angle = turnEncoder.getAbsolutePosition().getValue()*360;   
    SmartDashboard.putNumber("turn encoder", angle);
    
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

      setTurnMotor(MathUtil.clamp(speed, Constants.PIDControllers.TurnPID.PID_LOW_LIMIT, Constants.PIDControllers.TurnPID.PID_HIGH_LIMIT));
      // clamp and set speed
    }

  }
}