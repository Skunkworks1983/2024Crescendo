// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurnModuleTest;
import frc.robot.commands.testResetTurnMotorPID;
import frc.robot.constants.Constants;

public class OI extends SubsystemBase {
  private static OI oi;

  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttonStick;

  JoystickButton driveForward;

  JoystickButton switchMotors;
  JoystickButton testTurn;
    JoystickButton setTurnPID;

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);
    testTurn = new JoystickButton(rightJoystick,0);
    setTurnPID = new JoystickButton(leftJoystick,0);
    driveForward = new JoystickButton(buttonStick, 11);
    setTurnPID.onTrue(new testResetTurnMotorPID());
    driveForward.onTrue(new TurnModuleTest(90.0));
  }


  @Override

  public void periodic() {
  }

  public double getLeftX() { // Used to control the x field relative speed of the robot in SwerveTeleop.
    return -leftJoystick.getX(); 
  }

  public double getLeftY() { // Used to control the y field relative speed of the robot in SwerveTeleop.
    if(-leftJoystick.getY() <= 0.1 && driveForward.getAsBoolean()){
      return -1;
    }
    else{
    return -leftJoystick.getY(); 
    }  
  }

  public double getRightX() { // Used to control the rotational speed of the robot in SwerveTeleop.
    return -rightJoystick.getX();
  }

  public double getRightY() {
    return -rightJoystick.getY();
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}