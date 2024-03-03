// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

/** 
 * Used for keeping robot heading in the right direction using PID and the
 * targeting buttion 
*/
public class HeadingController {

    Drivebase drivebase;
    SmartPIDController headingPIDController;
    private static HeadingController headingController;
    double headingControllerSetpoint;
    double currentHeading;

    public HeadingController () {

        // PID Controller to control the robot's heading during SwerveTeleop
        headingPIDController = new SmartPIDController(
            Constants.PIDControllers.HeadingControlPID.KP, Constants.PIDControllers.HeadingControlPID.KI,
            Constants.PIDControllers.HeadingControlPID.KD, "Heading Controller",
            Constants.PIDControllers.HeadingControlPID.SMART_PID_ACTIVE);

        headingControllerSetpoint = 0.0;
        currentHeading = 0.0;
    }

    public void setDriveWithHeadingControl(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {
        double degreesPerSecond = headingPIDController.calculate(drivebase.getGyroAngle());
        drivebase.setDrive(xFeetPerSecond, yFeetPerSecond, degreesPerSecond, fieldRelative);
    }

    public void setHeadingSetpoint(double setpoint) {
        headingPIDController.setSetpoint(setpoint);
        SmartDashboard.putNumber("Heading Setpoint", setpoint);
    }

    public static HeadingController getInstance() {
        if (headingController == null) {
            headingController = new HeadingController();
        }

        return headingController;
    }
}
