// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.CLIMB_MODULE;
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
  }

  public void setClimberPosition(CLIMB_MODULE module, double setpointMeters) {
    leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    if (module == CLIMB_MODULE.LEFT) {
      leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    } 
    if (module == CLIMB_MODULE.RIGHT) {
      rightClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    }
  }

  public double getClimberPostition(CLIMB_MODULE module) {
    if (module == CLIMB_MODULE.LEFT) {
      return leftClimbMotor.getPosition().getValueAsDouble();
    } 
    if (module == CLIMB_MODULE.RIGHT) {
      return rightClimbMotor.getPosition().getValueAsDouble();
    }   

    return 0.0;
  }

  public double getClimberTorque(CLIMB_MODULE module) {
    if (module == CLIMB_MODULE.LEFT) {
      return leftClimbMotor.getTorqueCurrent().getValueAsDouble();
    } 
    if (module == CLIMB_MODULE.RIGHT) {
      return rightClimbMotor.getTorqueCurrent().getValueAsDouble();
    }
    
    return 0.0;
  }

  public void setClimberOutput(CLIMB_MODULE module, double percentOutput) {
    if (module == CLIMB_MODULE.LEFT) {
      leftClimbMotor.set(percentOutput);
    } 
    if (module == CLIMB_MODULE.RIGHT) {
      rightClimbMotor.set(percentOutput);
    }
  }

  public void setBrakeMode(CLIMB_MODULE module) {
    if (module == CLIMB_MODULE.LEFT) {
      leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    } 
    if (module == CLIMB_MODULE.RIGHT) {
      rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    }
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
