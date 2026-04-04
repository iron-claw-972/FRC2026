package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static double MAX_ANGLE = 180; // Deg
    public static double MIN_ANGLE = -200; // Deg

    public static double CALIBRATION_OFFSET = 0.0; // TODO: find this at hardstop

    public static double MAX_VELOCITY = 600; // rad/s
    // public static double MAX_ACCELERATION = 120.0; // rad/s^2
    public static double MAX_ACCELERATION = 320.0; // rad/s^2

    // Not using this, but just in case
    public static double TURRET_WIDTH = Units.inchesToMeters(6.4);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double GEAR_RATIO = 28;

    // Turret is in center of robot, but make use of the height in shooter physics
    public static Translation3d DISTANCE_FROM_ROBOT_CENTER = new Translation3d(0,0, Units.inchesToMeters(22.172)); //meters

	public static final double EXTRAPOLATION_TIME_CONSTANT = 0.06;

	public static final double FEEDFORWARD_KV = 0.06;

    public static final double NORMAL_CURRENT_LIMIT = 60.0; // A
    public static final double CALIBRATION_CURRENT_LIMIT = 10.0; // A
    public static final double CALIBRATION_CURRENT_THRESHOLD = 9.0; // A

}
