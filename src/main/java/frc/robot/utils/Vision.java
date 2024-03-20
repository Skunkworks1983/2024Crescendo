// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhotonVision;

public class Vision {
    SkunkPhotonCamera[] cameras;

    public Vision(SkunkPhotonCamera[] cameras) {
        this.cameras = cameras;
    }

    /**
     * Returns an ArrayList of VisionMeasurements. In drivebase, call addVisionMeasurements for each
     * item in this list every loop.
     */
    public ArrayList<VisionMeasurement> getLatestVisionMeasurements() {
        ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement>();
        int i = 1;

        // Iterate through list of cameras.
        for (SkunkPhotonCamera camera : cameras) {

            Optional<EstimatedRobotPose> updatedVisualPose = camera.poseEstimator.update();
            PhotonPipelineResult result = camera.camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
            Transform3d distanceToTargetTransform;

            SmartDashboard.putBoolean("Camera " + i + " hasTargets", hasTargets);

            // Check if there are targets
            if (updatedVisualPose.isPresent() && hasTargets) {

                // try/catch statement to ensure getBestCameraToTarget() won't crash code
                try {
                    distanceToTargetTransform = result.getBestTarget().getBestCameraToTarget();
                } catch (NullPointerException e) {
                    continue;
                }

                // Calculate the uncertainty of the vision measurement based on the distance
                // from the best AprilTag target.
                EstimatedRobotPose pose = updatedVisualPose.get();

                double distanceToTarget = Math.sqrt(Math.pow(distanceToTargetTransform.getX(), 2)
                        + Math.pow(distanceToTargetTransform.getY(), 2));

                double distanceUncertainty =
                        distanceToTarget * PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL;
                double rotationalUncertainty =
                        distanceToTarget * PhotonVision.ROTATIONAL_UNCERTAINTY_PROPORTIONAL;

                if (pose.estimatedPose.toPose2d().getX() < 0.0
                        || pose.estimatedPose.getX() > Constants.FIELD_X_LENGTH
                        || pose.estimatedPose.getY() < 0.0
                        || pose.estimatedPose.getY() > Constants.FIELD_Y_LENGTH
                        || distanceToTarget > PhotonVision.APRILTAG_DISTANCE_CUTOFF) {
                    continue;
                }

                SmartDashboard.putNumber("Camera " + i + " distanceToTarget", distanceToTarget);

                Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(new SimpleMatrix(new double[] {
                        distanceUncertainty, distanceUncertainty, rotationalUncertainty}));

                visionMeasurements.add(new VisionMeasurement(pose, uncertainty));
            }

            i++;
        }

        return visionMeasurements;
    }
}
