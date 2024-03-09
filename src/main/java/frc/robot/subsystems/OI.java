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
import frc.robot.commands.CollectorStow;
import frc.robot.commands.LowerCollector;
import frc.robot.commands.ManualIntakeNotes;
import frc.robot.commands.ManualRunNoteBackwards;
import frc.robot.commands.NoteFloorToShooter;
import frc.robot.commands.ResetGyroHeading;
import frc.robot.commands.ResetCollector;
import frc.robot.commands.SetFieldTarget;
import frc.robot.commands.shooter.AimShooterAtSpeakerAssumingNoGravity;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootWhenReady;
import frc.robot.commands.shooter.ShooterToAmp;
import frc.robot.commands.shooter.ShooterToAngle;
import frc.robot.commands.shooter.ShooterToPodium;
import frc.robot.commands.shooter.ShooterToStow;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.constants.Constants.Targeting.FieldTarget;

public class OI extends SubsystemBase {
  private static OI oi;

  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttonStick;

  JoystickButton manualSwitch;

  JoystickButton targetingSpeaker;
  JoystickButton targetingAmp;
  JoystickButton flywheelSpinup;
  JoystickButton linearAim;
  JoystickButton shootWhenReady;
  JoystickButton noteFloorToShooter;
  JoystickButton shooterToAmp;
  JoystickButton shooterToSpeaker;
  JoystickButton collectorStow;
  JoystickButton collectorDown;
  JoystickButton manualExpelBackwards;
  JoystickButton resetCollector;

  // Climber buttons
  JoystickButton smartClimb;
  JoystickButton manualLeftClimberUp;
  JoystickButton manualLeftClimberDown;
  JoystickButton manualRightClimberUp;
  JoystickButton manualRightClimberDown;

  JoystickButton resetGyroHeadingLeft;
  JoystickButton resetGyroHeadingRight;

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);

    manualSwitch = new JoystickButton(buttonStick, Constants.IDS.MANUAL_SWITCH);

    // Targeting buttons
    targetingSpeaker = new JoystickButton(rightJoystick, Constants.IDS.SPEAKER_TARGETING_BUTTON);
    targetingAmp = new JoystickButton(rightJoystick, Constants.IDS.AMP_TARGETING_BUTTON);

    // Shooter Pivot Buttons
    shooterToAmp = new JoystickButton(buttonStick, Constants.IDS.SHOOTER_TO_AMP);
    shooterToSpeaker = new JoystickButton(buttonStick, Constants.IDS.SHOOTER_TO_SPEAKER);

    linearAim = new JoystickButton(rightJoystick, Constants.IDS.LINEAR_AIM);

    flywheelSpinup = new JoystickButton(buttonStick, Constants.IDS.FLYWHEEL_SPINUP);
    shootWhenReady = new JoystickButton(buttonStick, Constants.IDS.SHOOT_WHEN_READY);

    collectorDown = new JoystickButton(buttonStick, Constants.IDS.COLLECTOR_DOWN);
    collectorStow = new JoystickButton(buttonStick, Constants.IDS.COLLECTOR_STOW);

    noteFloorToShooter = new JoystickButton(buttonStick, Constants.IDS.NOTE_FLOOR_TO_SHOOTER);
    manualExpelBackwards = new JoystickButton(buttonStick, 16);

    smartClimb = new JoystickButton(buttonStick, Constants.IDS.SMART_CLIMB);

    manualLeftClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_UP);
    manualLeftClimberDown = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_DOWN);
    manualRightClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_UP);
    manualRightClimberDown =
        new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_DOWN);

    resetCollector = new JoystickButton(buttonStick, 8);

    targetingSpeaker.whileTrue(new SetFieldTarget(FieldTarget.SPEAKER));
    targetingAmp.whileTrue(new SetFieldTarget(FieldTarget.AMP));

    shooterToAmp.whileTrue(new ShooterToAmp());
    shooterToAmp.negate().and(shooterToSpeaker.negate()).whileTrue(new ShooterToStow());

    resetGyroHeadingLeft = new JoystickButton(leftJoystick, Constants.IDS.RESET_GYRO_BUTTON);
    resetGyroHeadingRight = new JoystickButton(rightJoystick, Constants.IDS.RESET_GYRO_BUTTON);

    resetGyroHeadingLeft = new JoystickButton(leftJoystick, Constants.IDS.RESET_GYRO_BUTTON);
    resetGyroHeadingRight = new JoystickButton(rightJoystick, Constants.IDS.RESET_GYRO_BUTTON);

    shooterToSpeaker.whileTrue(new ShooterToPodium());
    flywheelSpinup.whileTrue(new FlywheelSpinup());
    shootWhenReady.whileTrue(new ShootWhenReady());

    collectorDown.whileTrue(new LowerCollector());
    collectorStow.whileTrue(new CollectorStow());

    noteFloorToShooter.whileTrue(new NoteFloorToShooter());
    manualExpelBackwards.and(manualSwitch).whileTrue(new ManualRunNoteBackwards());

    smartClimb.onTrue(new ExtendClimber());
    smartClimb.onFalse(new SmartClimb());

    manualLeftClimberUp.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.LEFT, .2));
    manualLeftClimberDown.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.LEFT, -.2));
    manualRightClimberUp.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, .2));
    manualRightClimberDown.and(manualSwitch)
        .whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, -.2));


    // Calling this command for both buttons to elimate confusion about which button needs to be
    // pressed first.
    resetGyroHeadingLeft.onTrue(new ResetGyroHeading(resetGyroHeadingLeft::getAsBoolean,
        resetGyroHeadingRight::getAsBoolean));
    resetGyroHeadingRight.onTrue(new ResetGyroHeading(resetGyroHeadingLeft::getAsBoolean,
        resetGyroHeadingRight::getAsBoolean));
        
    resetCollector.whileTrue(new ResetCollector());
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
