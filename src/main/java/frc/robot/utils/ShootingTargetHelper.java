// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

/** Add your docs here. */
public class ShootingTargetHelper {

    public Drivebase drivebase = Drivebase.getInstance();

    public ShootingTargetHelper() {

    }

    // Takes in a location to target to, and returns the initial velocity of the note
    public Translation3d calculateNoteInitialVelocity(Translation3d targetLocation) {

        // calculates the notes initial z position using conservation of energy
        double noteInitialVelocityZ =
                Math.sqrt(Math.pow(Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY, 2)
                        + 2 * (Constants.SpeakerTargetingMath.GRAVITY) * (targetLocation.getZ()
                                - Constants.SpeakerTargetingMath.AVG_SHOOTER_Z_POS));

        // calculates time to final position using the z starting velocity and z final velocity
        double t = (noteInitialVelocityZ - Constants.SpeakerTargetingMath.Z_FINAL_VELOCITY)
                / (Constants.SpeakerTargetingMath.GRAVITY);

        // calculates Initial Velocity for X and Y using the time
        double noteInitialVelocityY = (targetLocation.getY() - drivebase.getRobotPose().getY()) / t;
        double noteInitialVelocityX = (targetLocation.getX() - drivebase.getRobotPose().getX()) / t;

        return new Translation3d(noteInitialVelocityX, noteInitialVelocityY, noteInitialVelocityZ);
    }

    public double calculateShooterAngle(Translation3d targetLocation) {
 
        Translation3d NoteInitialVelocity = calculateNoteInitialVelocity(targetLocation);

        // 
        return Math.asin(NoteInitialVelocity.getZ()
                / (Math.hypot(NoteInitialVelocity.getX(), NoteInitialVelocity.getY())));
    }

    public double calculateFlywheelSpeed(Translation3d targetLocation) {
        Translation3d NoteInitialVelocity = calculateNoteInitialVelocity(targetLocation);
        return Math.sqrt(Math.pow(NoteInitialVelocity.getZ(), 2)
                + Math.pow(Math.hypot(NoteInitialVelocity.getX(), NoteInitialVelocity.getY()), 2));
    }
}
