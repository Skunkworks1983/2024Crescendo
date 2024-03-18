package frc.robot.utils;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class SkunkPhotonCamera {
    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout aprilTagFieldLayout;
    public String cameraName;

    public SkunkPhotonCamera(String cameraName, Transform3d transform3d) {
        this.cameraName = cameraName;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Exception reading AprilTag Field JSON " + e.toString());
        }

        // Creates a new PhotonCamera and PhotonPoseEstimator
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, transform3d);

    }
}
