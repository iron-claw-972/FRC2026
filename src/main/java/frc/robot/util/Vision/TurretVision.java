package frc.robot.util.Vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.VisionConstants;

public class TurretVision {

    private final PhotonCamera camera;
    private final Supplier<Transform3d> cameraLocationSupplier;

    public TurretVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
        cameraLocationSupplier = () -> new Transform3d(
            new Translation3d(
                VisionConstants.TURRET_CAMERA_X_OFFSET,
                VisionConstants.TURRET_CAMERA_Y_OFFSET,
                VisionConstants.TURRET_CAMERA_Z_OFFSET),
            new Rotation3d(0, VisionConstants.TURRET_CAMERA_PITCH, VisionConstants.TURRET_CAMERA_ROLL));
    }

    public TurretVision(String cameraName, Supplier<Transform3d> cameraLocationSupplier) {
        camera = new PhotonCamera(cameraName);
        this.cameraLocationSupplier = cameraLocationSupplier;
    }

    public Transform3d getCameraLocation() {
        return cameraLocationSupplier.get();
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
