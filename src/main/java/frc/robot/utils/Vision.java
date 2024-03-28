// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhotonVision;

public class Vision {
    SkunkPhotonCamera[] cameras;
    String [] hasTargetsPrints;
    String [] distanceToTargetPrints; 

    public Vision(SkunkPhotonCamera[] cameras) {
        this.cameras = cameras;
        hasTargetsPrints = new String[this.cameras.length];
        distanceToTargetPrints = new String[this.cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            hasTargetsPrints[i] = "Camera " + (i+1) + " hasTargets";
            distanceToTargetPrints[i] = "Camera " + (i+1) + " distanceToTarget";
        }
    }

    /**
     * Returns an ArrayList of VisionMeasurements. In drivebase, call addVisionMeasurements for each
     * item in this list every loop.
     */
    public ArrayList<VisionMeasurement> getLatestVisionMeasurements() {
        ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement>();
        int i = 0;

        // Iterate through list of cameras.
        for (SkunkPhotonCamera camera : cameras) {

            Optional<EstimatedRobotPose> updatedVisualPose = camera.poseEstimator.update();
            PhotonPipelineResult result = camera.camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
            Transform3d distanceToTargetTransform;

            SmartDashboard.putBoolean(hasTargetsPrints[i], hasTargets);

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

                // If the estimated pose is not within the field dimensions, then don't add the
                // vision measurement.
                if (pose.estimatedPose.toPose2d().getX() < 0.0
                        || pose.estimatedPose.toPose2d().getX() > Constants.FIELD_X_LENGTH
                        || pose.estimatedPose.toPose2d().getY() < 0.0
                        || pose.estimatedPose.toPose2d().getY() > Constants.FIELD_Y_LENGTH) {
                    continue;
                }

                SmartDashboard.putNumber(distanceToTargetPrints[i], distanceToTarget);

                Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(new SimpleMatrix(new double[] {
                        distanceUncertainty, distanceUncertainty, rotationalUncertainty}));

                // If the timestamp is in the future, then use the FPGA timestamp instead. Bad
                // timestamps will break the pose estimator.
                double timestamp = Math.min(pose.timestampSeconds, Timer.getFPGATimestamp());

                visionMeasurements.add(new VisionMeasurement(pose.estimatedPose.toPose2d(), uncertainty, timestamp));
            }

            i++;
        }

        return visionMeasurements;
    }
}
