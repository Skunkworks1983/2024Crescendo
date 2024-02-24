// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    leftController = new SmartPIDControllerTalonFX(
        Constants.PIDControllers.DrivePID.KP,
        Constants.PIDControllers.DrivePID.KI, 
        Constants.PIDControllers.DrivePID.KD,
        Constants.PIDControllers.DrivePID.KF, 
        "Climb Controller 1",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, leftClimbMotor);

    rightController = new SmartPIDControllerTalonFX(
        Constants.ClimberConstants.CLIMBER_KP,
        Constants.ClimberConstants.CLIMBER_KI, 
        Constants.ClimberConstants.CLIMBER_KD,
        Constants.ClimberConstants.CLIMBER_KF, 
        "Climb Controller 2",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, 
        );
  }

  public void setClimberPosition(boolean useLeftMotor, double setPointMeters) {
    TalonFX motor;
    if (useLeftMotor) {
      motor = leftClimbMotor;
    } else {
      motor = rightClimbMotor;
    }
    motor.setControl(postitionVoltage.withPosition(setPointMeters));
  }

  public void setLeftClimberPosition(double setpointMeters) {
    leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
  }

  public void setRightClimberPosition() {
    rightClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
  }

  public double getClimber1Postition() {
    return climbMotor1.getPosition().getValueAsDouble();
  }

  public double getClimber2Position() {
    return climbMotor2.getPosition().getValueAsDouble();
  }

  public double getClimber1Torque() {
    return climbMotor1.getTorqueCurrent().getValueAsDouble();
  }

  public double getClimber2torque() {
    return climbMotor2.getTorqueCurrent().getValueAsDouble();
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
