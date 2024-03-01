// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {
    public EstimatedRobotPose pose;
    public Matrix<N3, N1> stdDevs;

    public VisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
        this.pose = pose;
        this.stdDevs = stdDevs;
    }

}
