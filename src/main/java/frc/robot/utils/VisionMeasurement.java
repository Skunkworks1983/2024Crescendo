// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {
    public final Pose2d estimatedPose;
    public final Matrix<N3, N1> stdDevs;
    public final double timestamp;

    /**
     * An object that contains a EstimatedRobotPose, a Matrix of standard deviations, and a
     * timestamp.
     */
    public VisionMeasurement(Pose2d estimatedPose, Matrix<N3, N1> stdDevs, double timestamp) {
        this.estimatedPose = estimatedPose;
        this.stdDevs = stdDevs;
        this.timestamp = timestamp;
    }

}
