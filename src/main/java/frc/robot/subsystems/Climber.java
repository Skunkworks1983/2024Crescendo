// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Climber extends SubsystemBase {

  TalonFX leftClimbMotor;
  TalonFX rightClimbMotor;

  private static Climber climber;
  private final PositionVoltage postitionVoltage = new PositionVoltage(0);

  SmartPIDControllerTalonFX leftPositionController;
  SmartPIDControllerTalonFX rightPositionController;

  private Climber() {

    // Initialize the climbing motors.
    leftClimbMotor = new TalonFX(Constants.IDS.LEFT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);
    rightClimbMotor = new TalonFX(Constants.IDS.RIGHT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);

    TalonFXConfiguration inverted = new TalonFXConfiguration();
    TalonFXConfiguration notInverted = new TalonFXConfiguration();
    inverted.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    notInverted.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftClimbMotor.getConfigurator().apply(notInverted);
    rightClimbMotor.getConfigurator().apply(inverted);

    // Using Smart PID Controllers on the motors for position control
    leftPositionController = new SmartPIDControllerTalonFX(
        Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI,
        Constants.PIDControllers.ClimberPID.CLIMBER_KD, Constants.PIDControllers.DrivePID.KF,
        "Left Climb Motor", Constants.PIDControllers.ClimberPID.SMART_PID_ACTIVE, leftClimbMotor);

    rightPositionController = new SmartPIDControllerTalonFX(
        Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI,
        Constants.PIDControllers.ClimberPID.CLIMBER_KD, Constants.PIDControllers.DrivePID.KF,
        "Right Climb Motor", Constants.PIDControllers.ClimberPID.SMART_PID_ACTIVE, rightClimbMotor);

    SmartDashboard.putNumber("Left Climber Position", getClimberPostition(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Climber Position", getClimberPostition(ClimbModule.RIGHT));
    SmartDashboard.putNumber("Left Amps", getClimberTorque(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Amps", getClimberTorque(ClimbModule.RIGHT));
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
