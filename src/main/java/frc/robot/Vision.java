// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class Vision {
    AprilTagFieldLayout aprilTagFieldLayout;
    Vision.Camera[] visionUnits;

    public Vision(Vision.Camera[] cameras) {
        visionUnits = cameras;
    }

    public static class Camera {
        PhotonCamera camera;
        PhotonPoseEstimator poseEstimator;

        public Camera(String cameraName, Transform3d transform3d) {
            camera = new PhotonCamera(cameraName);
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                transform3d);
        }
    }

    /** Class that holds vision measurement data, such as EstimatedRobotPose and uncertainty standard devs. */
    public class VisionMeasurement {
        public EstimatedRobotPose pose;
        public Matrix<N3, N1> stdDevs;
        
        public VisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = stdDevs;
        }
    }

    /** Update odometry position. Call this function every loop in periodic. */
    public ArrayList<Vision.VisionMeasurement> getLatestVisionMeasurements() {
        ArrayList<Vision.VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement> ();

        for (Vision.Camera visionUnit : visionUnits) {

            Optional<EstimatedRobotPose> updatedVisualPose = visionUnit.poseEstimator.update();
            PhotonPipelineResult result = visionUnit.camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
            Transform3d distanceToTargetTransform;

            // Check if there are targets
            if (updatedVisualPose.isPresent() && hasTargets) {

                // try/catch statement to ensure getBestCameraToTarget() won't crash code
                try {
                    distanceToTargetTransform = result.getBestTarget().getBestCameraToTarget();
                } catch (NullPointerException e) {
                    return visionMeasurements;
                }

                // Calculate the uncertainty of the vision measurement based on distance from the
                // best
                // AprilTag target.
                EstimatedRobotPose pose = updatedVisualPose.get();
                double distanceToTarget = Math.sqrt(Math.pow(distanceToTargetTransform.getX(), 2)
                        + Math.pow(distanceToTargetTransform.getY(), 2));
                SmartDashboard.putNumber("Distance to target", distanceToTarget);
                Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(new SimpleMatrix(new double[] {
                        distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
                        distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
                        distanceToTarget
                                * Constants.PhotonVision.ROTATIONAL_UNCERTAINTY_PROPORTIONAL}));
                
                visionMeasurements.add(new VisionMeasurement(pose, uncertainty));
            }

            
        }
        return visionMeasurements;
    }
}
