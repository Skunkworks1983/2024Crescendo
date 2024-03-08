// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.robot.constants.Constants;

public class ShooterAimUtils {

  public static double calculateIdealStationaryShooterPivotAngle(Translation3d shooterPivotPosition,
      double flywheelSpeed) {
    double robotAngle = Math.atan2(
            Constants.Targeting.FieldTarget.SPEAKER_LOWEST_GOAL_PART.get().get().getY() - shooterPivotPosition.getY(),
            Constants.Targeting.FieldTarget.SPEAKER_LOWEST_GOAL_PART.get().get().getX() - shooterPivotPosition.getX());

    double distanceToSpeakerZ = 
      Constants.Targeting.FieldTarget.SPEAKER_LOWEST_GOAL_PART.get().get().getZ()
      - shooterPivotPosition.getZ();

    double distanceToSpeakerHoodZ = Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get().getZ() - 
    shooterPivotPosition.getZ();

    double distanceToSpeakerH = calculateHorizontalDistance(shooterPivotPosition,
        Constants.Targeting.FieldTarget.SPEAKER_LOWEST_GOAL_PART.get().get());

        //Speaker
        double minimumAngle = hitPositionToMinAngle(Constants.Shooter.ANGLE_SEARCH_DEPTH,
        distanceToSpeakerH,
            0.0, Math.PI/4.0, distanceToSpeakerZ, flywheelSpeed);

            //Hood
    double maximumAngle = hitPositionToMinAngle(Constants.Shooter.ANGLE_SEARCH_DEPTH,
        distanceToSpeakerH - (Constants.Targeting.distanceFromHoodToSpeaker / Math.cos(robotAngle)), 0.0, Math.PI/4.0, distanceToSpeakerHoodZ,
        flywheelSpeed);
        
    double desiredAngle = (maximumAngle * (Constants.Shooter.AUTO_AIM_ROTATION_RATIO)
        + minimumAngle * (1.0 - Constants.Shooter.AUTO_AIM_ROTATION_RATIO));
    return desiredAngle;
  }

  // meters/second
  public static double calculateIdealFlywheelSpeed(Translation2d shooterPivotTranslation) {
    return Constants.Shooter.BASE_FLYWHEEL_AUTOAIMING_SPEED
        + (Constants.Targeting.FieldTarget.SPEAKER_HOOD.get().get().toTranslation2d()
            .getDistance(shooterPivotTranslation)
            * Constants.Shooter.BASE_FLYWHEEL_AUTOAIMING_SPEED_PER_METER_DISTANCE);
  }

  // Based on math from
  // https://www.chiefdelphi.com/t/angled-shooter-math-analysis/455087
  static double noteHitPosition(double theta, double distance, double velocity) {
    double t = (distance/(velocity*Math.cos(theta)))-(Constants.Shooter.PIVOT_TO_FLYWHEEL_DISTANCE)/velocity;
    return (-.5*Constants.ACCELERATION_DUE_TO_GRAVITY*
    Math.pow(t,2))+(velocity*Math.sin(theta)*t)+(Constants.Shooter.PIVOT_TO_FLYWHEEL_DISTANCE)*Math.sin(theta);
  }

  // uses bisection to find pivot rotation based on angle of shooter
  // https://en.wikipedia.org/wiki/Bisection_method
  static double hitPositionToMinAngle(int depth,double distance, double minInput, double maxInput,
      double desiredHitPosition, double flywheelSpeed) {
    double pivotAngleGuess = (minInput + maxInput) / 2.0;
    for (int i = 0; i < depth; i++) {

      boolean tooLow = (desiredHitPosition > noteHitPosition(pivotAngleGuess, desiredHitPosition,
          flywheelSpeed));
      if (tooLow) {
        minInput = pivotAngleGuess;
      } else {
        maxInput = pivotAngleGuess;
      }
      pivotAngleGuess = (minInput + maxInput) / 2.0;
    }
    return pivotAngleGuess;
  }

  // in meters
  // horizontal means x and y but no z
  public static double calculateHorizontalDistance(Translation3d a, Translation3d b) {
    return a.toTranslation2d().getDistance(b.toTranslation2d());
  }

  /*
   * all rotations are in radians drivebase translation is field reletive.
   * PivotTranslation is
   * drivebase reletive This math uses matricies to change reference frames.
   * Because pivot is our
   * final position and a reference frame, we convert 0,0,0 pivot reletive to
   * field reletive.
   * 
   * This code precomputes trigonometry before the matricies because each of the
   * variables are used
   * mulitple times. *
   */
  public static Translation3d calculatePivotPositionFieldReletive(double drivebaseRotation,
      double shooterPivotRotation, Translation2d drivebaseTranslation) {
    Translation3d pivotTranslation = Constants.Shooter.ROBOT_RELATIVE_PIVOT_POSITION;
    double cosD = Math.cos(drivebaseRotation);
    double sinD = Math.sin(drivebaseRotation);

    // theta - 90 is neccecary to convert from the system in which forward is 90 and
    // up is 0 to the
    // system in which 0 is forward and 90 is upward.
    double cosP = Math.cos(Math.PI / 2.0 - shooterPivotRotation);
    double sinP = Math.sin(Math.PI / 2.0 - shooterPivotRotation);

    Matrix<N4, N4> robotToField = new Matrix<N4, N4>(
        new SimpleMatrix(new double[][] { { cosD, -sinD, 0.0, drivebaseTranslation.getX() },
            { sinD, cosD, 0.0, drivebaseTranslation.getY() }, { 0.0, 0.0, 1.0, 0.0 },
            { 0.0, 0.0, 0.0, 1.0 } }));

    Matrix<N4, N4> pivotToRobot = new Matrix<N4, N4>(new SimpleMatrix(new double[][] {
        { cosP, 0.0, sinP, pivotTranslation.getX() }, { 0.0, 1.0, 0.0, pivotTranslation.getY() },
        { -sinP, 0.0, cosP, pivotTranslation.getZ() }, { 0.0, 0.0, 0.0, 1.0 } }));
    Matrix<N4, N1> zeroCoordinate = new Matrix<N4, N1>(new SimpleMatrix(new double[] { 0.0, 0.0, 0.0, 1.0 }));

    Matrix<N4, N1> fieldPositionOfPivot = robotToField.times(pivotToRobot.times(zeroCoordinate));
    return new Translation3d(fieldPositionOfPivot.get(0, 0), fieldPositionOfPivot.get(1, 0),
        fieldPositionOfPivot.get(2, 0));
  }
}
