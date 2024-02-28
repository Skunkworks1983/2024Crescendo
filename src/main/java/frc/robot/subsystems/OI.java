// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.ManualMoveClimber;
import frc.robot.commands.climber.RetractClimber;
import frc.robot.commands.climber.SmartClimb;
import frc.robot.commands.LowerCollector;
import frc.robot.commands.ManualIntakeNotes;
import frc.robot.commands.SetFieldTarget;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.Shoot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.constants.Constants.Targeting.FieldTarget;

public class OI extends SubsystemBase {
  private static OI oi;

  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttonStick;
  JoystickButton targetingSpeaker;
  JoystickButton targetingAmp;
  JoystickButton switchMotors;
  JoystickButton manualIntakeNotes;
  JoystickButton flywheelSpinup;
  JoystickButton manualShoot;
  JoystickButton collectorPositionChange;

  // Climber buttons
  JoystickButton extendClimber;
  JoystickButton retractClimber;
  JoystickButton smartClimb;
  JoystickButton manualLeftClimberUp;
  JoystickButton manualLeftClimberDown;
  JoystickButton manualRightClimberUp;
  JoystickButton manualRightClimberDown;

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);

    // Targeting buttons
    targetingSpeaker = new JoystickButton(rightJoystick, Constants.IDS.SPEAKER_TARGETING_BUTTON);
    targetingAmp = new JoystickButton(rightJoystick, Constants.IDS.AMP_TARGETING_BUTTON);

    manualIntakeNotes = new JoystickButton(buttonStick, Constants.IDS.MANUAL_PERCENT_OUTPUT);
    flywheelSpinup = new JoystickButton(buttonStick, Constants.IDS.FLYWHEEL_SPINUP);
    manualShoot = new JoystickButton(buttonStick, Constants.IDS.MANUAL_SHOOT);
    collectorPositionChange = new JoystickButton(buttonStick, Constants.IDS.COLLECTOR_POSITION_CHANGE);

    extendClimber = new JoystickButton(buttonStick, Constants.IDS.EXTEND_CLIMBER);
    retractClimber = new JoystickButton(buttonStick, Constants.IDS.RETRACT_CLIMBER);
    smartClimb = new JoystickButton(buttonStick, Constants.IDS.SMART_CLIMB);

    manualLeftClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_UP);
    manualLeftClimberDown = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_DOWN);
    manualRightClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_UP);
    manualRightClimberDown = new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_DOWN);

    targetingSpeaker.whileTrue(new SetFieldTarget(FieldTarget.SPEAKER));
    targetingAmp.whileTrue(new SetFieldTarget(FieldTarget.AMP));

    manualIntakeNotes.whileTrue(new ManualIntakeNotes());
    flywheelSpinup.whileTrue(new FlywheelSpinup());
    manualShoot.whileTrue(new Shoot());

    extendClimber.onTrue(new ExtendClimber());
    retractClimber.onTrue(new RetractClimber());
    smartClimb.onTrue(new SmartClimb());
    manualLeftClimberUp.whileTrue(new ManualMoveClimber(ClimbModule.LEFT, .05));
    manualLeftClimberDown.whileTrue(new ManualMoveClimber(ClimbModule.LEFT, -.05));
    manualRightClimberUp.whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, .05));
    manualRightClimberDown.whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, -.05));

    collectorPositionChange.whileTrue(new LowerCollector());
  }

  @Override
  public void periodic() {}

  // Used to control the x field relative speed of the robot in SwerveTeleop.
  public double getLeftX() {

    // Positive joystick corrosponds to negaive robot relative coordinates, so leftJoystick.getX()
    // must be negated.
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

    return -rightJoystick.getY();
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
