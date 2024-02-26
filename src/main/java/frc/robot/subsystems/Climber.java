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
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.utils.SmartPIDControllerTalonFX;

// This is a stub subsystem
public class Climber extends SubsystemBase {

  TalonFX leftClimbMotor;
  TalonFX rightClimbMotor;

  private static Climber climber;
  private final PositionVoltage postitionVoltage = new PositionVoltage(0);

  /** Creates a new Climber. */
  private Climber() {

    // Initialize the climbing motors.
    leftClimbMotor = new TalonFX(Constants.IDS.LEFT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);
    rightClimbMotor = new TalonFX(Constants.IDS.LEFT_CLIMBER_MOTOR, Constants.CANIVORE_NAME);
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
      return leftClimbMotor.getTorqueCurrent().getValueAsDouble();
    }
    if (module == ClimbModule.RIGHT) {
      return rightClimbMotor.getTorqueCurrent().getValueAsDouble();
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
