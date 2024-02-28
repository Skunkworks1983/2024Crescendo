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

// This is a stub subsystem
public class Climber extends SubsystemBase {

  TalonFX leftClimbMotor;
  TalonFX rightClimbMotor;

  private static Climber climber;
  private final PositionVoltage postitionVoltage = new PositionVoltage(0);
  SmartPIDControllerTalonFX leftPositionController;
  SmartPIDControllerTalonFX rightPositionController;

  /** Creates a new Climber. */
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

    leftPositionController = new SmartPIDControllerTalonFX(Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI, Constants.PIDControllers.ClimberPID.CLIMBER_KD,
        Constants.PIDControllers.DrivePID.KF, "Left Climb Motor",
        false, leftClimbMotor);
    
    rightPositionController = new SmartPIDControllerTalonFX(Constants.PIDControllers.ClimberPID.CLIMBER_KP,
        Constants.PIDControllers.ClimberPID.CLIMBER_KI, Constants.PIDControllers.ClimberPID.CLIMBER_KD,
        Constants.PIDControllers.DrivePID.KF, "Right Climb Motor",
        false, rightClimbMotor);

    SmartDashboard.putNumber("Left Climber Position", getClimberPostition(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Climber Position", getClimberPostition(ClimbModule.RIGHT));
    SmartDashboard.putNumber("Left Amps", climber.getClimberTorque(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Amps", climber.getClimberTorque(ClimbModule.RIGHT));
  }

  public void setClimberPosition(ClimbModule module, double setpointMeters) {
    leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    }
    if (module == ClimbModule.RIGHT) {
      rightClimbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
    }
  }

  public double getClimberPostition(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      return leftClimbMotor.getPosition().getValueAsDouble();
    }
    if (module == ClimbModule.RIGHT) {
      return rightClimbMotor.getPosition().getValueAsDouble();
    }

    return 0.0;
  }

  public double getClimberTorque(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      return Math.abs(leftClimbMotor.getTorqueCurrent().getValueAsDouble());
    }
    if (module == ClimbModule.RIGHT) {
      return Math.abs(rightClimbMotor.getTorqueCurrent().getValueAsDouble());
    }

    return 0.0;
  }

  public void setClimberOutput(ClimbModule module, double percentOutput) {
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.set(percentOutput);
    }
    if (module == ClimbModule.RIGHT) {
      rightClimbMotor.set(percentOutput);
    }
  }

  public void setBrakeMode(ClimbModule module) {
    if (module == ClimbModule.LEFT) {
      leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    if (module == ClimbModule.RIGHT) {
      rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public boolean atPositionSetpoint(ClimbModule module, double setpoint) {
    if (Math.abs(getClimberPostition(module) - setpoint) < ClimberConstants.CLIMBER_POSITION_TOLERANCE) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Position", getClimberPostition(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Climber Position", getClimberPostition(ClimbModule.RIGHT));
    SmartDashboard.putNumber("Left Amps", climber.getClimberTorque(ClimbModule.LEFT));
    SmartDashboard.putNumber("Right Amps", climber.getClimberTorque(ClimbModule.RIGHT));
  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }
}
