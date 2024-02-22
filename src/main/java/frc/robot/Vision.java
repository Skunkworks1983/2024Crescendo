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
    AprilTagFieldLayout aprilTagFieldLayout;

    public Vision (Vision.Camera [] cameras) {
    }
    
    public class Camera {
        public Camera(String cameraName, Transform3d transform3d) {
            PhotonCamera camera = new PhotonCamera(cameraName);
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras.get(cameras.size() - 1),
                transform3d);
        }
    }

    public void getLatestVisionMeasurements() {
        PhotonPipelineResult result;
        boolean hasTargets;

        for(PhotonCamera i : cameras) {
            result = i.getLatestResult();
            hasTargets = result.hasTargets();
        }
        
        for (PhotonPoseEstimator i : visualOdometries) {
            Optional<EstimatedRobotPose> updatedPose = i.update();
            if (updatedPose.isPresent() && hasTargets) {

            }
        }
    }
}
