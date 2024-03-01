package frc.robot.vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants.PhotonVision.PipelineType;

public class SkunkPhotonCamera {
    PhotonCamera camera;
    String cameraName;
    Transform3d transform3d;
    PipelineType pipelineType;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout aprilTagFieldLayout;

    public SkunkPhotonCamera(String cameraName, Transform3d transform3d, PipelineType pipelineType) {

        this.cameraName = cameraName;
        this.transform3d = transform3d;
        this.pipelineType = pipelineType;

        // Creates a new PhotonCamera
        camera = new PhotonCamera(cameraName);

        if (pipelineType == PipelineType.APRILTAG) {

            try {
                aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            } catch (IOException e) {
                System.out.println("Exception reading AprilTag Field JSON " + e.toString());
            }

            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, transform3d);
        }
    }
}
