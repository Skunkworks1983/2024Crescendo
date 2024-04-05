// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.constants.Constants.ClimberConstants.ClimberSlotConfigs;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Climber extends SubsystemBase {

  TalonFX leftClimbMotor;
  TalonFX rightClimbMotor;

  private static Climber climber;
  private final PositionVoltage postitionVoltage = new PositionVoltage(0);

  SmartPIDControllerTalonFX leftClimbPositionController;
  SmartPIDControllerTalonFX rightClimbPositionController;

  Slot1Configs slot1configs;

  private Climber() {

    // Initialize the climbing motors.
    leftClimbMotor = new TalonFX(Constants.IDS.LEFT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);
    rightClimbMotor = new TalonFX(Constants.IDS.RIGHT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);

    TalonFXConfiguration inverted = new TalonFXConfiguration();
    TalonFXConfiguration notInverted = new TalonFXConfiguration();
    inverted.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    notInverted.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftClimbMotor.getConfigurator().apply(inverted);
    rightClimbMotor.getConfigurator().apply(notInverted);

    // Using Smart PID Controllers on the motors for position control
    leftClimbPositionController = new SmartPIDControllerTalonFX(
        Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI,
        Constants.PIDControllers.ClimberPID.CLIMBER_KD, Constants.PIDControllers.DrivePID.KF,
        "Left Climb Motor", Constants.PIDControllers.ClimberPID.SMART_PID_ACTIVE, leftClimbMotor);

    rightClimbPositionController = new SmartPIDControllerTalonFX(
        Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI,
        Constants.PIDControllers.ClimberPID.CLIMBER_KD, Constants.PIDControllers.DrivePID.KF,
        "Right Climb Motor", Constants.PIDControllers.ClimberPID.SMART_PID_ACTIVE, rightClimbMotor);

      slot1configs = new Slot1Configs();

      slot1configs.kP = .6;
      slot1configs.kI = 0.0;
      slot1configs.kD = 0.0;
      slot1configs.kV = 0.0;

    //SmartDashboard.putNumber("Left Climber Position", getClimberPostition(ClimbModule.LEFT));
    //SmartDashboard.putNumber("Right Climber Position", getClimberPostition(ClimbModule.RIGHT));
    //SmartDashboard.putNumber("Left Amps", getClimberTorque(ClimbModule.LEFT));
    //SmartDashboard.putNumber("Right Amps", getClimberTorque(ClimbModule.RIGHT));
  }

  public void applySlotConfig(ClimbModule module, ClimberSlotConfigs config) {
    if (module == ClimbModule.LEFT) {
      if (config == ClimberSlotConfigs.SLOT_0) {
        leftClimbMotor.getConfigurator().apply(leftClimbPositionController.slot0Configs);
      } else {
        leftClimbMotor.getConfigurator().apply(slot1configs);
      }
    } else {
      if (config == ClimberSlotConfigs.SLOT_0) {
        rightClimbMotor.getConfigurator().apply(rightClimbPositionController.slot0Configs);
      } else {
        rightClimbMotor.getConfigurator().apply(slot1configs);
      }
    }
  }

  public void setClimberPosition(ClimbModule module, double setpointMeters) {
    leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    } else {
      rightClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    }
  }

  public double getClimberPostition(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      return leftClimbMotor.getPosition().getValueAsDouble();
    } else {
      return rightClimbMotor.getPosition().getValueAsDouble();
    }
  }

  public double getClimberTorque(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      return Math.abs(leftClimbMotor.getTorqueCurrent().getValueAsDouble());
    } else {
      return Math.abs(rightClimbMotor.getTorqueCurrent().getValueAsDouble());
    }
  }

  public void setClimberOutput(ClimbModule module, double percentOutput) {
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.set(percentOutput);
    } else {
      rightClimbMotor.set(percentOutput);
    }
  }

  public void setBrakeMode(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public boolean atPositionSetpoint(double setpoint, ClimbModule module) {
    SmartDashboard.putNumber("within tolerance", Math.abs(getClimberPostition(module) - setpoint));
    return Math.abs(getClimberPostition(module) - setpoint) < ClimberConstants.CLIMBER_MOTOR_POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {

    // Updating SmartDashboard prints every loop.
    //SmartDashboard.putNumber("Left Climber Position", getClimberPostition(ClimbModule.LEFT));
    //SmartDashboard.putNumber("Right Climber Position", getClimberPostition(ClimbModule.RIGHT));
    //SmartDashboard.putNumber("Left Amps", climber.getClimberTorque(ClimbModule.LEFT));
    //SmartDashboard.putNumber("Right Amps", climber.getClimberTorque(ClimbModule.RIGHT));
  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }
}
