package frc.robot.util.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.GyroBiasConstants;

/**
 * estimates gyro bias by comparing vision-derived yaw to gyro yaw.
 *
 * when the robot observes April tags, PhotonVision calculates what the robot heading
 * SHOULD be based on known tag positions vs observed angles. This can be compared to
 * the gyro reading to detect and correct drift.
 * 
 */
public class GyroBiasEstimator {

    private double weightedBiasSum = 0.0;
    private double totalWeight = 0.0;
    private int sampleCount = 0;
    
    // exponential moving average
    private double emaBias = 0.0;
    private boolean emaInitialized = false;

    /**
     * process a new observation comparing vision pose to gyro reading.
     *
     * @param visionPose the pose estimated by vision (from PhotonVision)
     * @param gyroYaw current gyro reading in radians
     * @param visionWeight weight for observation (0.0 to 1.0, higher is more trusted)
     * @return true if bias should be applied (has enough samples)
     */
    public boolean addObservation(Pose3d visionPose, double gyroYaw, double visionWeight) {
        if (visionPose == null) {
            return false;
        }

        // get yaw from vision
        double visionYaw = visionPose.getRotation().getZ();

        return addObservation(visionYaw, gyroYaw, visionWeight);
    }

    /**
     * process a new observation with just yaw values.
     *
     * @param visionYaw yaw from vision pose in radians
     * @param gyroYaw current gyro reading in radians
     * @param visionWeight weight for this observation (0.0 to 1.0, higher is more trusted)
     * @return true if bias should be applied
     */
    public boolean addObservation(double visionYaw, double gyroYaw, double visionWeight) {
        // normalize to [-PI, PI]
        double diff = normalizeAngle(visionYaw - gyroYaw);

        // reject outliers
        if (Math.abs(diff) > GyroBiasConstants.MAX_ANGLE_DIFF_RAD) {
            return false;
        }

        // clamp weight
        double weight = Math.max(0.0, Math.min(1.0, visionWeight));
        
        // accumulate weighted bias
        weightedBiasSum += diff * weight;
        totalWeight += weight;
        sampleCount++;

        // update exponential moving average
        if (!emaInitialized) {
            emaBias = diff;
            emaInitialized = true;
        } else {
            emaBias = emaBias * (1.0 - GyroBiasConstants.EMA_ALPHA) + diff * GyroBiasConstants.EMA_ALPHA;
        }

        return sampleCount >= GyroBiasConstants.MIN_SAMPLES;
    }

    /**
     * process new observation with default weight of 1.0.
     * maintains backward compatibility.
     */
    public boolean addObservation(Pose3d visionPose, double gyroYaw) {
        return addObservation(visionPose, gyroYaw, 1.0);
    }

    /**
     * process new observation with default weight of 1.0.
     * maintains backward compatibility
     */
    public boolean addObservation(double visionYaw, double gyroYaw) {
        return addObservation(visionYaw, gyroYaw, 1.0);
    }

    /**
     * get average bias and reset
     *
     * @return average bias in radians to apply, or 0 if not enough samples
     */
    public double getAndResetBias() {
        if (sampleCount < GyroBiasConstants.MIN_SAMPLES || totalWeight < GyroBiasConstants.MIN_TOTAL_WEIGHT) {
            return 0.0;
        }

        // use weighted average if we have enough weight, otherwise use EMA
        double avgBias;
        if (totalWeight >= GyroBiasConstants.MIN_TOTAL_WEIGHT) {
            avgBias = weightedBiasSum / totalWeight;
        } else {
            avgBias = emaBias;
        }

        // reset
        weightedBiasSum = 0.0;
        totalWeight = 0.0;
        sampleCount = 0;
        emaInitialized = false;

        return avgBias;
    }

    /**
     * apply partial correction to avoid sudden jumps.
     *
     * @param fullBias the full calculated bias
     * @return partial correction to apply
     */
    public double applyPartialCorrection(double fullBias) {
        double clampedBias = fullBias;
        if (clampedBias > GyroBiasConstants.MAX_CORRECTION_PER_CYCLE_RAD) {
            clampedBias = GyroBiasConstants.MAX_CORRECTION_PER_CYCLE_RAD;
        } else if (clampedBias < -GyroBiasConstants.MAX_CORRECTION_PER_CYCLE_RAD) {
            clampedBias = -GyroBiasConstants.MAX_CORRECTION_PER_CYCLE_RAD;
        }

        return clampedBias * GyroBiasConstants.CORRECTION_FRACTION;
    }

    /**
     * normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        return MathUtil.angleModulus(angle);
    }

    /**
     * get sample count for debugging
     */
    public int getSampleCount() {
        return sampleCount;
    }

    /**
     * get current accumulated bias without resetting
     */
    public double getCurrentBias() {
        if (sampleCount == 0) {
            return 0.0;
        }
        if (totalWeight > 0) {
            return weightedBiasSum / totalWeight;
        }
        return emaBias;
    }

    /**
     * get current total weight for debugging
     */
    public double getTotalWeight() {
        return totalWeight;
    }

    /**
     * reset accumulated state
     */
    public void reset() {
        weightedBiasSum = 0.0;
        totalWeight = 0.0;
        sampleCount = 0;
        emaInitialized = false;
    }
}
