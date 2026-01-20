package frc.robot.util.Vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class TurretVision {

    private final PhotonCamera camera;

    public TurretVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

  /**
   * @param tagId AprilTag ID to aim at
   * @return Optional yaw error in radians (positive = target to the right, negative = target to the left)
   */
    public Optional<Double> getYawToTagRad(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return Optional.empty();
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagId) {
                // PhotonVision yaw is in DEGREES
                double yawRad = Units.degreesToRadians(target.getYaw());
                return Optional.of(yawRad);
            }
        }

        return Optional.empty();
    }

    public boolean canSeeTag(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) return false;

        for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == tagId) {
            return true;
        }
        }
        return false;
    }
}
