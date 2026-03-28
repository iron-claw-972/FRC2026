package frc.robot.constants;

/**
 * constants for gyro bias estimation and correction via vision.
 */
public class GyroBiasConstants {
    /** minimum samples before applying correction */
    public static final int MIN_SAMPLES = 10;

    /** maximum angle difference to accept in radians */
    public static final double MAX_ANGLE_DIFF_RAD = Math.toRadians(45);

    /** minimum correction to apply in radians */
    public static final double MIN_CORRECTION_RAD = Math.toRadians(1.0);

    /** fraction of the correction to apply (0.0 to 1.0) */
    public static final double CORRECTION_FRACTION = 0.2;

    /** maximum correction per cycle in radians */
    public static final double MAX_CORRECTION_PER_CYCLE_RAD = 0.05; // 2.9 deg

    /** alpha for exponential moving average 0.0 to 1.0, higher is more responsive */
    public static final double EMA_ALPHA = 0.3;

    /** min total weight required for weighted average */
    public static final double MIN_TOTAL_WEIGHT = 3.0;
}
