package frc.robot.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class PieceData {

    double yaw;
    double pitch;
    double area;
    double skew;
    Transform3d bestCameraToTargetTransform;

    public PieceData(PhotonTrackedTarget target) {
        this.yaw = target.getYaw();
        this.pitch = target.getPitch();
        this.area = target.getArea();
        this.skew = target.getSkew();
        this.bestCameraToTargetTransform = target.getBestCameraToTarget();
    }

}
