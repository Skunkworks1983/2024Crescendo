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
  JoystickButton switchMotors;
  

  public OI() {
    leftJoystick = new Joystick(Constants.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
  }

  @Override

  public void periodic() {

  }

  public double getLeftX() { // Used to control the x field relative speed of the robot in SwerveTeleop.

    double leftXValue = leftJoystick.getX();

    System.out.println("leftX joystick value " + leftXValue);

    return leftXValue;
  }

  public double getLeftY() { // Used to control the y field relative speed of the robot in SwerveTeleop.

    double leftYValue = leftJoystick.getY();   // maybe reverse this to make it go in right direction

    System.out.println("leftY joystick value " + leftYValue);

    return leftYValue;
  }

  public double getRightX() { // Used to control the rotational speed of the robot in SwerveTeleop.

    double rightXValue = rightJoystick.getX();

    System.out.println("rightX joystick value " + rightXValue);

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