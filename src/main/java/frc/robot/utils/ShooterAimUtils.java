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
import frc.robot.constants.Constants.ShooterInterpolationConstants;

public class ShooterAimUtils {

  public static double calculateHorizontalDistance(Translation3d a, Translation3d b) {
    return a.toTranslation2d().getDistance(b.toTranslation2d());
  }



  public static double calculateInterpolatedAimAngle(double x, double y) {
    return calculateInterpolatedAimAngle(new Translation2d(x, y));
  }

  // based on https://www.desmos.com/3d/3d22c872bf
  public static double calculateInterpolatedAimAngle(Translation2d robotPosition) {

    // simple average (not used but included so it makes more sense to people who read this in the
    // future)
    double total = 0;
    double numberOfValues = 0;

    // weighted based on distance
    // You can think of it as there are effectively more copies of that point if position
    // value is close to that point.
    double weightedNumberOfValues = 0;
    double weightedTotal = 0;

    for (Translation3d point : Constants.ShooterInterpolationConstants.KNOWN_SHOOTING_POINTS) {
      double weight =
          calculateWeightBasedOnDistance(point.toTranslation2d().getDistance(robotPosition));
      numberOfValues += 1;
      total += point.getZ();
      weightedNumberOfValues += weight;
      weightedTotal += weight * point.getZ();
    }

    double unweightedAverage = total / numberOfValues;
    double weightedAverage = weightedTotal / weightedNumberOfValues;
    return weightedAverage;
  }

  // critical equasion. exactly what values we feed into it dictate how well the mesh fits to the mesh
  private static double calculateWeightBasedOnDistance(double distance) {
    return ((ShooterInterpolationConstants.DISTANCE_TO_WEIGHT_A_1 * Math.pow(2,
        -(Math.pow(ShooterInterpolationConstants.DISTANCE_TO_WEIGHT_B_1 * distance, 2))))
        + (ShooterInterpolationConstants.DISTANCE_TO_WEIGHT_A_2 * Math.pow(2,
            -(Math.pow(ShooterInterpolationConstants.DISTANCE_TO_WEIGHT_B_2 * distance, 2)))));
  }

  // only use the quarter(not actually 1/4 of area of the field) of the field close to 0,0
  // reflects across center of field on x and speaker y
  // will reflect point to be in the first quarter
  public static Translation2d calculateInputForInterpolatedAimAngle(
      Translation2d originalPosition) {
    Translation2d reflectionPosition = new Translation2d(Constants.FIELD_X_LENGTH / 2.0,
        Constants.Targeting.FieldTarget.SPEAKER.get().get().getY());
    double xDistanceFromReflectionLine =
        Math.abs(originalPosition.getX() - reflectionPosition.getX());
    double yDistanceFromReflectionLine =
        Math.abs(originalPosition.getY() - reflectionPosition.getY());
    double x = reflectionPosition.getX() - xDistanceFromReflectionLine;
    double y = reflectionPosition.getY() - yDistanceFromReflectionLine;
    Translation2d newPosition = new Translation2d(x, y);
    return newPosition;
  }

  /*
   * all rotations are in radians drivebase translation is field relative. PivotTranslation is
   * drivebase relative This math uses matricies to change reference frames. Because pivot is our
   * final position and a reference frame, we convert 0,0,0 pivot relative to field relative.
   * 
   * This code precomputes trigonometry before the matricies because each of the variables are used
   * mulitple times.
   */
  public static Translation3d calculatePivotPositionFieldRelative(double drivebaseRotation,
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
        new SimpleMatrix(new double[][] {{cosD, -sinD, 0.0, drivebaseTranslation.getX()},
            {sinD, cosD, 0.0, drivebaseTranslation.getY()}, {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}}));

    Matrix<N4, N4> pivotToRobot = new Matrix<N4, N4>(new SimpleMatrix(new double[][] {
        {cosP, 0.0, sinP, pivotTranslation.getX()}, {0.0, 1.0, 0.0, pivotTranslation.getY()},
        {-sinP, 0.0, cosP, pivotTranslation.getZ()}, {0.0, 0.0, 0.0, 1.0}}));
    Matrix<N4, N1> zeroCoordinate =
        new Matrix<N4, N1>(new SimpleMatrix(new double[] {0.0, 0.0, 0.0, 1.0}));

    Matrix<N4, N1> fieldPositionOfPivot = robotToField.times(pivotToRobot.times(zeroCoordinate));
    return new Translation3d(fieldPositionOfPivot.get(0, 0), fieldPositionOfPivot.get(1, 0),
        fieldPositionOfPivot.get(2, 0));
  }
}
