// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class SwerveModule extends SubsystemBase {

  TalonFX driveMotor;
  TalonFX turnMotor;
  CANcoder turnEncoder;
  String modulePosition;

  SmartPIDController turnController;
  SmartPIDControllerTalonFX driveController;

  final VelocityVoltage velocityController = new VelocityVoltage(0);

  public SwerveModule(Constants.SwerveModuleConstants swerveModuleConstants) {

    driveMotor = new TalonFX(swerveModuleConstants.driveMotorId, Constants.CANIVORE_NAME);
    turnMotor = new TalonFX(swerveModuleConstants.turnMotorId, Constants.CANIVORE_NAME);
    turnEncoder = new CANcoder(swerveModuleConstants.turnEncoderId, Constants.CANIVORE_NAME);
    this.modulePosition = swerveModuleConstants.modulePosition;

    turnController = new SmartPIDController(Constants.PIDControllers.TurnPID.KP,
        Constants.PIDControllers.TurnPID.KI, Constants.PIDControllers.TurnPID.KD,
        modulePosition + " Turn", Constants.PIDControllers.TurnPID.SMART_PID_ACTIVE);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = -swerveModuleConstants.turnEncoderOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(canCoderConfig);

    // Pid controller will loop from -180 to 180 continuously
    turnController.enableContinuousInput(-180, 180);

    // sets the tolerance of the turning pid controller.
    turnController.setTolerance(Constants.PIDControllers.TurnPID.TURN_PID_TOLERANCE);
    
    // reseting the configuration to default
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveMotor.getConfigurator().apply(driveConfig);
    turnMotor.getConfigurator().apply(turnConfig);
    driveController = new SmartPIDControllerTalonFX(Constants.PIDControllers.DrivePID.KP,
        Constants.PIDControllers.DrivePID.KI, Constants.PIDControllers.DrivePID.KD,
        Constants.PIDControllers.DrivePID.KF, modulePosition + " Drive",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, driveMotor);

    velocityController.Slot = 0;
  }

  @Override
  public void periodic() {
    updateTurnSpeedBasedOnSetpoint();
    driveController.updatePID();
  }

  // sets drive motor in velocity mode (set feet per second)
  public void setDriveMotorVelocity(double feetPerSecond) {

    double revsPerSecond = feetPerSecond * Constants.DrivebaseInfo.REVS_PER_FOOT;
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

    double feetPerSecond =
        driveMotor.getVelocity().getValue() / Constants.DrivebaseInfo.REVS_PER_FOOT;
    return feetPerSecond;
  }

  public SwerveModuleState getSwerveState() {
    return new SwerveModuleState(Units.feetToMeters(getDriveEncoderVelocity()),
        Rotation2d.fromDegrees(getTurnEncoder()));
  }

  /** gets turn encoder as degrees, -180 180 */
  public double getTurnEncoder() { // TODO: change from degrees to radians.
    // multiplying absolute postion by 360 to convert from +- .5 to +- 180
    // gets the absoulte position of the encoder. getPosition() returns relative
    // position.
    double angle = turnEncoder.getAbsolutePosition().getValue() * 360;
    SmartDashboard.putNumber("turn encoder", angle);

    return angle;
  }

  public void setPID(double p, double i, double d) {
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = p;
    slot0Configs.kI = i;
    slot0Configs.kD = d;

    driveMotor.getConfigurator().apply(slot0Configs);

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(Units.feetToMeters(getDriveEncoderPosition()),
        Rotation2d.fromDegrees(getTurnEncoder()));
  }

  public void setState(SwerveModuleState desiredState) {
    double turnPositionRadians = Units.degreesToRadians(getTurnEncoder());
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turnPositionRadians));

    // velocityScale helps prevent driving in the wrong direction when making sudden
    // turns.
    // cos(0)=1, so if module is in the right direction, there is no speed decrease.
    // cos(90)=0, so if module is completely off, the module will not drive at all.
    // this value is squared to increase its effects.
    double velocityScale =
        Math.pow(Math.cos(optimized.angle.getRadians() - (turnPositionRadians)), 2);

    double scaledVelocity = Units.metersToFeet( optimized.speedMetersPerSecond);//velocityScale)
    SmartDashboard.putNumber("setting velocity", scaledVelocity);
    setDriveMotorVelocity(scaledVelocity);

    // set setpoint
    turnController.setSetpoint(optimized.angle.getDegrees());

    SmartDashboard.putNumber("turn setPoint "+ modulePosition, optimized.angle.getDegrees());
    SmartDashboard.putNumber("drive setPoint "+modulePosition, scaledVelocity);
    SmartDashboard.putNumber("turn encoder " +modulePosition,  getTurnEncoder()); 
  }

  void updateTurnSpeedBasedOnSetpoint() {
    // calculate speed
    double speed = -turnController.calculate(getTurnEncoder());
    boolean atSetpoint = turnController.atSetpoint();

    if (!atSetpoint) {
      // clamp and set speed
      setTurnMotorSpeed(MathUtil.clamp(speed, Constants.PIDControllers.TurnPID.PID_LOW_LIMIT,
          Constants.PIDControllers.TurnPID.PID_HIGH_LIMIT));
    }
  }
}
