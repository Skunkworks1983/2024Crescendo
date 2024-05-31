// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

/** If each of the motors on the swerve module are running */
public class SwerveModuleStatus {

    public boolean isDriveRunning;
    public boolean isTurnRunning;

    public SwerveModuleStatus(boolean isDriveRunning, boolean isTurnRunning) {
        this.isDriveRunning = isDriveRunning;
        this.isTurnRunning = isTurnRunning;
    }
}
