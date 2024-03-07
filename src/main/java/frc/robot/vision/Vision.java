// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.constants.Constants.PhotonVision.PipelineType;

public class Vision {
    private static Vision vision;
    SkunkPhotonCamera[] cameras;

    private Vision(SkunkPhotonCamera[] cameras) {
        this.cameras = cameras;
    }

    /**
     * Returns an Optional PieceData object. Returns Optional.empty() if:
     * <ul><li>No camera exists with the specified camera name.</li></ul>
     * <ul><li>The camera with the specified camera name is not being used for piece detection.</li></ul>
     * <ul><li>There are no targets detected by the camera.</li></ul>
     *  
    */
    public Optional<PieceData> getPieceData(String pieceDetectionCameraName) {
        Optional<PieceData> pieceData = Optional.empty();

        // Iterate through each camera
        for (SkunkPhotonCamera camera : cameras) {

            // If statement to check if it is the specified camera
            if (camera.cameraName == pieceDetectionCameraName && camera.pipelineType == PipelineType.PIECE_DETECTION) {
                PhotonPipelineResult result = camera.camera.getLatestResult();

                // If statement to verify whether the latest result from the camera has any targets to prevent
                // code from crashing.
                if (result.hasTargets()) {
                    PhotonTrackedTarget target = result.getBestTarget();
                    pieceData = Optional.of(new PieceData(target));
                }
            }
        }

        return pieceData;
    }

    /**
     * Returns an ArrayList of VisionMeasurements. In drivebase, call
     * addVisionMeasurements for each item in this list every loop.
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
                SmartDashboard.putNumber("Camera " + i + " distanceToTarget",
                        distanceToTarget);
                Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(new SimpleMatrix(new double[] {
                        distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
                        distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
                        distanceToTarget
                                * Constants.PhotonVision.ROTATIONAL_UNCERTAINTY_PROPORTIONAL }));

                visionMeasurements.add(new VisionMeasurement(pose, uncertainty));
            }

            i++;
        }

        return visionMeasurements;
    }

    public static Vision getInstance() {
        if (vision == null) {
            try {
                vision = new Vision(new SkunkPhotonCamera[] {
                    new SkunkPhotonCamera(PhotonVision.CAMERA_1_NAME, PhotonVision.ROBOT_TO_CAMERA_1, PipelineType.APRILTAG),
                    new SkunkPhotonCamera(PhotonVision.CAMERA_2_NAME, PhotonVision.ROBOT_TO_CAMERA_2, PipelineType.APRILTAG) });
                SmartDashboard.putBoolean(PhotonVision.CAMERA_STATUS_BOOLEAN, true);
            } catch (Exception e) {
                System.out.println("Exception creating cameras: " + e.toString());
                vision = new Vision(new SkunkPhotonCamera[] {});
                SmartDashboard.putBoolean(PhotonVision.CAMERA_STATUS_BOOLEAN, false);
            }
        }

        return vision;
    }
}
