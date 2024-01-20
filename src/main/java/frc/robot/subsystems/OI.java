// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants;

public class OI extends SubsystemBase {
  private static OI oi;

  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttonStick;

  JoystickButton switchMotors;
  

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);
  }


  @Override

  public void periodic() {
  }

  public double getLeftX() { // Used to control the x field relative speed of the robot in SwerveTeleop.

    return leftJoystick.getX(); 

  }

  public double getLeftY() { // Used to control the y field relative speed of the robot in SwerveTeleop.
  return leftJoystick.getY(); 
  }

  public double getRightX() { // Used to control the rotational speed of the robot in SwerveTeleop.
    return rightJoystick.getX();
  }

  public double getRightY() {
    return rightJoystick.getY();
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}