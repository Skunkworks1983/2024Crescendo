// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.ManualMoveClimber;
import frc.robot.commands.climber.SmartClimb;
import frc.robot.commands.CollectorStow;
import frc.robot.commands.LowerCollector;
import frc.robot.commands.ManualRunNoteBackwards;
import frc.robot.commands.NoteFloorToShooter;
import frc.robot.commands.SetFieldTarget;
import frc.robot.commands.SetRobotRelativeSwerve;
import frc.robot.commands.TestMechanicalOdometry;
import frc.robot.commands.autoAmp;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.IntakeShooterFromSource;
import frc.robot.commands.shooter.ShootWhenReady;
import frc.robot.commands.shooter.ShooterToAmp;
import frc.robot.commands.shooter.ShooterToPassAngle;
import frc.robot.commands.shooter.ShooterToStow;
import frc.robot.commands.shooter.tuningCommands.InterpolationAimShooterCommand;
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
  JoystickButton interpolationAim;
  JoystickButton shootWhenReady;
  JoystickButton noteFloorToShooter;
  JoystickButton shooterToAmp;
  JoystickButton collectorStow;
  JoystickButton collectorDown;
  JoystickButton manualExpelBackwards;
  JoystickButton resetCollector;
  JoystickButton shooterToPass;

  // Climber buttons
  JoystickButton smartClimb;
  JoystickButton manualLeftClimberUp;
  JoystickButton manualLeftClimberDown;
  JoystickButton manualRightClimberUp;
  JoystickButton manualRightClimberDown;

  JoystickButton setRobotRelitive;

  JoystickButton resetGyroHeadingLeft;
  JoystickButton resetGyroHeadingRight;

  // JoystickButton testMechanicalOdometry;
  
  // Shooter intake button
  JoystickButton shooterIntake;

  public OI() {
    leftJoystick = new Joystick(Constants.IDS.LEFT_JOYSTICK);
    rightJoystick = new Joystick(Constants.IDS.RIGHT_JOYSTICK);
    buttonStick = new Joystick(Constants.IDS.BUTTON_STICK);

    manualSwitch = new JoystickButton(buttonStick, Constants.IDS.MANUAL_SWITCH);

    setRobotRelitive = new JoystickButton(rightJoystick, Constants.IDS.SET_ROBOT_RELATIVE);

    // Targeting buttons
    targetingSpeaker = new JoystickButton(rightJoystick, Constants.IDS.SPEAKER_TARGETING_BUTTON);
    targetingAmp = new JoystickButton(rightJoystick, Constants.IDS.AMP_TARGETING_BUTTON);

    // Shooter Pivot Buttons
    shooterToAmp = new JoystickButton(buttonStick, Constants.IDS.SHOOTER_TO_AMP);
    shooterToPass = new JoystickButton(buttonStick, Constants.IDS.SHOOTER_TO_PASS);

    interpolationAim = new JoystickButton(buttonStick, Constants.IDS.INTERPOLATION_AIM);

    flywheelSpinup = new JoystickButton(buttonStick, Constants.IDS.FLYWHEEL_SPINUP);
    shootWhenReady = new JoystickButton(buttonStick, Constants.IDS.SHOOT_WHEN_READY);

    collectorDown = new JoystickButton(buttonStick, Constants.IDS.COLLECTOR_DOWN);
    collectorStow = new JoystickButton(buttonStick, Constants.IDS.COLLECTOR_STOW);

    noteFloorToShooter = new JoystickButton(buttonStick, Constants.IDS.NOTE_FLOOR_TO_SHOOTER);
    manualExpelBackwards = new JoystickButton(buttonStick, Constants.IDS.REVERSE_NOTE_BACKWARDS);

    smartClimb = new JoystickButton(buttonStick, Constants.IDS.SMART_CLIMB);

    manualLeftClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_UP);
    manualLeftClimberDown = new JoystickButton(buttonStick, Constants.IDS.MANUAL_LEFT_CLIMBER_DOWN);
    manualRightClimberUp = new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_UP);
    manualRightClimberDown =
        new JoystickButton(buttonStick, Constants.IDS.MANUAL_RIGHT_CLIMBER_DOWN);

    resetGyroHeadingLeft = new JoystickButton(leftJoystick, Constants.IDS.RESET_GYRO_BUTTON);
    resetGyroHeadingRight = new JoystickButton(rightJoystick, Constants.IDS.RESET_GYRO_BUTTON);
    resetCollector = new JoystickButton(buttonStick, Constants.IDS.RESET_COLLECTOR);

    // testMechanicalOdometry = new JoystickButton(rightJoystick, 6);
    
    shooterIntake = new JoystickButton(buttonStick, Constants.IDS.SHOOTER_INTAKE);

    setRobotRelitive.whileTrue(new SetRobotRelativeSwerve());

    targetingSpeaker.whileTrue(new SetFieldTarget(FieldTarget.SPEAKER));
    targetingAmp.whileTrue(new autoAmp(this));

    shooterToAmp.whileTrue(new ShooterToAmp());
    shooterToAmp.negate().and(interpolationAim.negate()).and(shooterToPass.negate()).and(shooterIntake.negate())
        .whileTrue(new ShooterToStow());
    shooterToPass.whileTrue(new ShooterToPassAngle());

    interpolationAim.whileTrue(new InterpolationAimShooterCommand());
    flywheelSpinup.whileTrue(new FlywheelSpinup());
    shootWhenReady.whileTrue(new ShootWhenReady());

    collectorDown.whileTrue(new LowerCollector());
    collectorStow.whileTrue(new CollectorStow());

    noteFloorToShooter.whileTrue(new NoteFloorToShooter());
    manualExpelBackwards.whileTrue(new ManualRunNoteBackwards());

    smartClimb.onTrue(new ExtendClimber());
    smartClimb.onFalse(new SmartClimb());

    manualLeftClimberUp.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.LEFT, .2));
    manualLeftClimberDown.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.LEFT, -.2));
    manualRightClimberUp.and(manualSwitch).whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, .2));
    manualRightClimberDown.and(manualSwitch)
        .whileTrue(new ManualMoveClimber(ClimbModule.RIGHT, -.2));
    resetGyroHeadingLeft.and(resetGyroHeadingRight).onTrue(new ResetGyro());

    // testMechanicalOdometry.whileTrue(new TestMechanicalOdometry());

    shooterIntake.whileTrue(new IntakeShooterFromSource());
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
