// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerTalonFX;

// This is a stub subsystem
public class ClimbModule extends SubsystemBase {

  TalonFX climbMotor;

  final PositionVoltage postitionVoltage = new PositionVoltage(0);
  SmartPIDControllerTalonFX CLIMBER_PID_ACTIVE;
  SmartPIDControllerTalonFX leftController;
  SmartPIDControllerTalonFX rightController;

  /** Creates a new Climber. */
  private ClimbModule(int motorId) {

    // Initialize the climbing motors.
    climbMotor = new TalonFX(motorId, Constants.CANIVORE_NAME);

    // Smart PID controller
    rightController = new SmartPIDControllerTalonFX(Constants.ClimberConstants.CLIMBER_KP,
        Constants.ClimberConstants.CLIMBER_KI, Constants.ClimberConstants.CLIMBER_KD,
        Constants.ClimberConstants.CLIMBER_KF, "Right Climber",
        Constants.PIDControllers.DrivePID.SMART_PID_ACTIVE, climbMotor);
  }

  public void setClimberPosition(double setpointMeters) {
    climbMotor.setControl(postitionVoltage.withPosition(setpointMeters));
  }

  public double getClimberPostition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  public double getClimberTorque() {
    return climbMotor.getTorqueCurrent().getValueAsDouble();
  }

  public void setClimberOutput(double percentOutput) {
    climbMotor.set(percentOutput);
  }

  public void setBrakeMode() {
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
