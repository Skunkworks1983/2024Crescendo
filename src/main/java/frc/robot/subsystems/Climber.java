// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.utils.SmartPIDControllerTalonFX;

// This is a stub subsystem
public class Climber extends SubsystemBase {

  TalonFX leftClimbMotor;
  TalonFX rightClimbMotor;

  private static Climber climber;
  final PositionVoltage postitionVoltage = new PositionVoltage(0);
  SmartPIDControllerTalonFX CLIMBER_PID_ACTIVE;
  SmartPIDControllerTalonFX leftController;
  SmartPIDControllerTalonFX rightController;

  /** Creates a new Climber. */
  private Climber() {

    // Initialize the climbing motors.
    leftClimbMotor = new TalonFX(Constants.IDS.CLIMBER_MOTOR_1, Constants.CANIVORE_NAME);
    rightClimbMotor = new TalonFX(Constants.IDS.CLIMBER_MOTOR_2, Constants.CANIVORE_NAME);

    // Smart PID controllers
    leftController = new SmartPIDControllerTalonFX(
        Constants.ClimberConstants.CLIMBER_KP,
        Constants.ClimberConstants.CLIMBER_KI, 
        Constants.ClimberConstants.CLIMBER_KD,
        Constants.ClimberConstants.CLIMBER_KF, 
        "Left Climber",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, rightClimbMotor
        );

    rightController = new SmartPIDControllerTalonFX(
        Constants.ClimberConstants.CLIMBER_KP,
        Constants.ClimberConstants.CLIMBER_KI, 
        Constants.ClimberConstants.CLIMBER_KD,
        Constants.ClimberConstants.CLIMBER_KF, 
        "Right Climber",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, rightClimbMotor
        );
  }

  public void setLeftClimberPosition(double setpointMeters) {
    leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
  }

  public void setRightClimberPosition(double setpointMeters) {
    rightClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
  }

  public double getLeftClimberPostition() {
    return leftClimbMotor.getPosition().getValueAsDouble();
  }

  public double getRightClimberPosition() {
    return rightClimbMotor.getPosition().getValueAsDouble();
  }

  public double getLeftClimberTorque() {
    return leftClimbMotor.getTorqueCurrent().getValueAsDouble();
  }

  public double getRightClimberTorque() {
    return rightClimbMotor.getTorqueCurrent().getValueAsDouble();
  }

  public void setLeftClimberOutput(double percentOutput) {
    leftClimbMotor.set(percentOutput);
  }

  public void setRightClimberOutput(double percentOutput) {
    rightClimbMotor.set(percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }
}
