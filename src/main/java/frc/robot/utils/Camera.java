package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
        PhotonCamera camera;
        PhotonPoseEstimator poseEstimator;

        public Camera(String cameraName, Transform3d transform3d) {
            camera = new PhotonCamera(cameraName);
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                transform3d);
        }
}