// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class ShooterAimUtils {

 public static double calculateIdealStationaryShooterPivotAngle(Translation3d shooterPivotPosition,
            double flywheelSpeed) {
        double robotAngle = Math.atan2(
                shooterPivotPosition.getX()
                        - Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get().getX(),
                shooterPivotPosition.getY()
                        - Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get().getY());

        double distanceToHood = calculateHorizontalDistance(shooterPivotPosition,
                Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get());
        double maximumAngle = hitPositionToMinAngle(Constants.Shooter.ANGLE_SEARCH_DEPTH,
                distanceToHood, 0.0, Math.PI, flywheelSpeed);

        double minimumAngle = hitPositionToMinAngle(Constants.Shooter.ANGLE_SEARCH_DEPTH,
                distanceToHood
                        + (Math.cos(robotAngle) * Constants.Targeting.distanceFromHoodToSpeaker),
                0.0, Math.PI, flywheelSpeed);

        double desiredAngle = (maximumAngle * (Constants.Shooter.AUTO_AIM_ROTATION_RATIO)
                + minimumAngle * (1.0 - Constants.Shooter.AUTO_AIM_ROTATION_RATIO));
        return desiredAngle;
    }

    // meters/second
    public static double calculateIdealFlywheelSpeed(Translation3d shooterPivotTranslation) {
        return Constants.Shooter.BASE_FLYWHEEL_AUTOAIMING_SPEED + (calculateHorizontalDistance(
                Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get(), shooterPivotTranslation)
                * Constants.Shooter.BASE_FLYWHEEL_AUTOAIMING_SPEED_PER_METER_DISTANCE);
    }

    // Based on math from https://www.chiefdelphi.com/t/angled-shooter-math-analysis/455087
    static double noteHitPosition(double theta, double distance, double velocity) {
        double t = (distance - (Math.cos(theta) * Constants.Shooter.PIVOT_TO_FLYWHEEL_DISTANCE))
                / (Math.cos(theta) * velocity);
        return (-.5 * Constants.ACCELERATION_DUE_TO_GRAVITY * Math.pow(t, 2))
                + (velocity * Math.sin(theta) * t)
                + (Constants.Shooter.PIVOT_TO_FLYWHEEL_DISTANCE * Math.sin(theta));
    }

    // uses bisection to find pivot rotation based on angle of shooter
    // https://en.wikipedia.org/wiki/Bisection_method
    static double hitPositionToMinAngle(int depth, double minInput, double maxInput,
            double desiredHitPosition, double flywheelSpeed) {
        double pivotAngleGuess = (minInput + maxInput) / 2;
        for (int i = 0; i < depth; i++) {

            boolean tooLow = (desiredHitPosition > noteHitPosition(pivotAngleGuess,
                    desiredHitPosition, flywheelSpeed));
            if (tooLow) {
                minInput = pivotAngleGuess;
            } else {
                maxInput = pivotAngleGuess;
            }
        }
        return pivotAngleGuess;
    }

    static Translation3d positionDiff(Translation3d a, Translation3d b) {
        return new Translation3d(a.getX() - b.getX(), a.getY() - b.getY(), a.getZ() - b.getZ());
    }

    // in meters 
    // Horisontal means x and y but no z
    public static double calculateHorizontalDistance(Translation3d a, Translation3d b) {
        return calculateHorizontalDistance(a.toTranslation2d(), a.toTranslation2d());
    }

    public static double calculateHorizontalDistance(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX(), a.getY())
                .getDistance(new Translation2d(b.getX(), b.getY()));
    }
}
