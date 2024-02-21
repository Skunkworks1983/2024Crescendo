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
    ArrayList<EstimatedRobotPose> visionMeasurements;
    ArrayList<PhotonCamera> cameras;
    ArrayList<PhotonPoseEstimator> visualOdometries;

    PhotonCamera camera = new PhotonCamera(Constants.PhotonVision.PHOTON_CAMERA_NAME);
    PhotonPoseEstimator visualOdometry_1;
    AprilTagFieldLayout aprilTagFieldLayout;

    public Vision(String [] cameraNames, Transform3d cameraTransforms) {
        for(String i : cameraNames) {
            cameras.add(new PhotonCamera(Constants.PhotonVision.PHOTON_CAMERA_NAME));
        }

        for (String i : cameraTransforms) {
            visionOdometries.add()
        }

        visualOdometry_1 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                Constants.PhotonVision.ROBOT_TO_CAMERA);

    }

    public void getLatestVisionMeasurements() {


        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        // Check if there are targets.
        if (updatedVisualPose.isPresent() && hasTargets) {
            visionMeasurements.add(updatedVisualPose.get());
        }


        
        
    }
}
