// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetFieldTarget;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;

public class OI extends SubsystemBase {
  private static OI oi;

  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttonStick;
  JoystickButton targetingSpeaker;
  JoystickButton targetingAmp;
  JoystickButton switchMotors;  

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);

    // Targeting buttons
    targetingSpeaker = new JoystickButton(rightJoystick, Constants.IDS.SPEAKER_TARGETING_BUTTON);
    targetingAmp = new JoystickButton(rightJoystick, Constants.IDS.AMP_TARGETING_BUTTON);
    
    targetingSpeaker.whileTrue(new SetFieldTarget(FieldTarget.SPEAKER));
    targetingAmp.whileTrue(new SetFieldTarget(FieldTarget.AMP));
  }

  @Override
  public void periodic() {}

  // Used to control the x field relative speed of the robot in SwerveTeleop.
  public double getLeftX() {
    return -leftJoystick.getX();
  }

  // Used to control the y field relative speed of the robot in SwerveTeleop.
  public double getLeftY() {

    // Positive joystick corrosponds to negetive robot relative coordiantes, so leftJoystick.getY()
    // must be negated.
    return -leftJoystick.getY();
  }

  // Used to control the rotational speed of the robot in SwerveTeleop.
  public double getRightX() {

    // Positive joystick corrosponds to negative robot relative coordiantes, so rightJoystick.getX()
    // must be negated.
    return -rightJoystick.getX();
  }

  public double getRightY() {

    // Positive joystick corrosponds to negaive robot relative coordinates, so rightJoystick.getY()
    // must be negated.
    return rightJoystick.getY();
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
