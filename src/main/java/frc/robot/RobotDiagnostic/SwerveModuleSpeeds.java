// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

/** Speeds to run the module's motors at */
public class SwerveModuleSpeeds {
    public double driveSpeed;
    public double turnSpeed;

    public SwerveModuleSpeeds (double driveSpeed, double turnSpeed) {
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;
    }
}
