// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

/** Add your docs here. */
public class ShootingTargetHelper {

    public Drivebase drivebase = Drivebase.getInstance();

    public ShootingTargetHelper() {

    }

    public double shooterAngle(Pose3d targetLocation) {

        double noteInitialVelocityZ =
                Math.sqrt(Math.pow(Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY, 2)
                        + 2 * (9.81) * (targetLocation.getZ()
                                - Constants.SpeakerTargetingMath.AVG_SHOOTER_Z_POS));
        double t =
                (noteInitialVelocityZ - Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY) / (9.81);
        double noteInitialVelocityY = (targetLocation.getY() - drivebase.getRobotPose().getY()) / t;
        double noteInitialVelocityX = (targetLocation.getX() - drivebase.getRobotPose().getX()) / t;
        return Math.asin(
                noteInitialVelocityZ / (Math.hypot(noteInitialVelocityX, noteInitialVelocityY)));
    }

    public double flywheelSpeed(Pose3d targetLocation) {
        double noteInitialVelocityZ =
                Math.sqrt(Math.pow(Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY, 2)
                        + 2 * (9.81) * (targetLocation.getZ()
                                - Constants.SpeakerTargetingMath.AVG_SHOOTER_Z_POS));
        double t =
                (noteInitialVelocityZ - Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY) / (9.81);
        double noteInitialVelocityY = (targetLocation.getY() - drivebase.getRobotPose().getY()) / t;
        double noteInitialVelocityX = (targetLocation.getX() - drivebase.getRobotPose().getX()) / t;
        return Math.sqrt(Math.pow(noteInitialVelocityZ, 2)
                + Math.pow(Math.hypot(noteInitialVelocityX, noteInitialVelocityY), 2));
    }

    public double chassisRotation(Pose3d targetLocation) {
        // not used/not finished
        double noteInitialVelocityZ =
                Math.sqrt(Math.pow(Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY, 2)
                        + 2 * (9.81) * (targetLocation.getZ()
                                - Constants.SpeakerTargetingMath.AVG_SHOOTER_Z_POS));
        double t =
                (noteInitialVelocityZ - Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY) / (9.81);
        double noteInitialVelocityY = (targetLocation.getY() - drivebase.getRobotPose().getY()) / t;
        double noteInitialVelocityX = (targetLocation.getX() - drivebase.getRobotPose().getX()) / t;
        return 0;
    }
}
