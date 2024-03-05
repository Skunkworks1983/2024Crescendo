// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import java.util.Timer;

/** Used to contorl the robot's heading during SwerveTeleop. */
public class HeadingController {

    // This is updated to the robot's current heading when using the targeting
    // button or when outside the turn joystick deadzone. Used for heading
    // correction when not using the targeting button and when inside the turn
    // joystick deadzone.
    double currentHeading = 0.0;

    // Parts of the code set this variable, and then the variable is used to tell
    // the drive command that turns to a certan angle where to turn to.
    double headingControllerSetpoint = 0.0;

    // hasUpdated ensures that desired heading is only set once, when the driver
    // stops rotating. If it is false and the robot should maintain current heading,
    // desiredHeadingSetpoint will set to current heading. Once it is set to true,
    // the robot will rotate to desiredHeadingSetpoint.
    boolean hasUpdated = false;

    // Time at the last input
    double timeAtLastInput;

    Timer timer = new Timer();

    public HeadingController() {
        timeAtLastInput = timer.();


    }

    public void setHeadingControllerSetpoint () {

    }
}
