// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

// This is a stub subsystem
public class Climber extends SubsystemBase {

  TalonFX climbMotor1;
  TalonFX climbMotor2;
  
  private Slot0Configs Slot0Configs;

  private 

  private static Climber climber;
  

  /** Creates a new Climber. */
  public Climber() {
    Slot0Configs s0conf = new Slot0Configs();
    s0conf.kP = Constants.CLIMBER_KP;
    s0conf.kI = Constants.CLIMBER_KI;
    s0conf.kD = Constants.CLIMBER_KD;
    s0conf.kV = Constants.CLIMBER_KF;
    climbMotor1 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_1, Constants.CANIVORE_NAME);
    climbMotor2 = new TalonFX(Constants.IDS.CLIMBER_MOTOR_2, Constants.CANIVORE_NAME);
  }

  public void setLeftClimberPosition(){
    // setpoint
    
  }
  public void setRightClimberPosition(){
    
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
