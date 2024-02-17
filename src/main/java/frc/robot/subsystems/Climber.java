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

  TalonFX climbMotor1;
  TalonFX climbMotor2;
  
  private Slot0Configs s0conf;

  private static Climber climber;
  
  final PositionVoltage postitionVoltage = new PositionVoltage(0);

  SmartPIDControllerTalonFX CLIMBER_PID_ACTIVE;

  /** Creates a new Climber. */
  private Climber() {
    s0conf = new Slot0Configs();
    s0conf.kP = Constants.ClimberConstants.CLIMBER_KP;
    s0conf.kI = Constants.ClimberConstants.CLIMBER_KI;
    s0conf.kD = Constants.ClimberConstants.CLIMBER_KD;
    s0conf.kV = Constants.ClimberConstants.CLIMBER_KF;

    climbMotor1 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_1, Constants.CANIVORE_NAME);
    climbMotor2 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_2, Constants.CANIVORE_NAME);
    climbMotor1.getConfigurator().apply(s0conf);
  }

  public void setClimberPosition(boolean useLeftMotor, double setPointMeters){
    TalonFX motor;
    if(useLeftMotor){
     motor = climbMotor1;
    }
    else{
      motor = climbMotor2;
    }
    motor.setControl(postitionVoltage.withPosition(setPointMeters));
  }
public double getClimberPostition(boolean isLeftMotor){
  if(isLeftMotor){
    return climbMotor1.getPosition().getValueAsDouble();
  }
  else{
    return climbMotor2.getPosition().getValueAsDouble();
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
