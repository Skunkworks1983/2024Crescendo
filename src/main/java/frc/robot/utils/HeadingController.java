// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Used to contorl the robot's heading during SwerveTeleop. */
public class HeadingController {

  // This is updated to the robot's current heading when using the targeting
  // button or when outside the turn joystick deadzone. Used for heading
  // correction when not using the targeting button and when inside the turn
  // joystick deadzone.
  double currentHeading;

  // Parts of the code set this variable, and then the variable is used to tell
  // the drive command that turns to a certan angle where to turn to.
  double headingControllerSetpoint;

  // hasUpdated ensures that desired heading is only set once, when the driver
  // stops rotating. If it is false and the robot should maintain current heading,
  // desiredHeadingSetpoint will set to current heading. Once it is set to true,
  // the robot will rotate to desiredHeadingSetpoint.
  boolean hasUpdated;

  double milliseconds;

  // Time at the last input
  double timeOfLastInput;

  private final Timer timer = new Timer();

  SmartPIDController headingPIDController = new SmartPIDController(
      Constants.PIDControllers.HeadingControlPID.KP, Constants.PIDControllers.HeadingControlPID.KI,
      Constants.PIDControllers.HeadingControlPID.KD, "Heading Controller",
      Constants.PIDControllers.HeadingControlPID.SMART_PID_ACTIVE);

  public HeadingController() {
    headingPIDController.enableContinuousInput(0, 360);
    currentHeading = 0;
    headingControllerSetpoint = 0;
    hasUpdated = false;


    timer.scheduleAtFixedRate(new TimerTask() {
      @Override
      public void run() {
        milliseconds += 1;
      }

    }, 1, 1);
  }

  /**
   * Calculate the rotational velocity of the robot when using the heading controller.
   */
  public double calculate(double headingMeasurment) {
    double degreesPerSecond = headingPIDController.calculate(headingMeasurment);
    return degreesPerSecond;
  }

  /** Sets the setpoint of the heading controller. */
  public void setSetpoint(double setpoint) {
    headingPIDController.setSetpoint(setpoint);
    SmartDashboard.putNumber("Heading Controller Setpoint", setpoint);
  }

  public void setTimeOfLastInput() {
    timeOfLastInput = milliseconds;
  }

  public boolean useHeadingControl() {
    return Units.millisecondsToSeconds(
        milliseconds - timeOfLastInput) >= Constants.TIME_UNTIL_HEADING_CONTROL;
  }
}
