package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static double MAX_ANGLE = 180;
    public static double MIN_ANGLE = -180;

    public static double MAX_VELOCITY = 10000000; // m/s
    public static double MAX_ACCELERATION = 10000000; // m/s^2

    public static double TURRET_WIDTH = Units.feetToMeters(1.0);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double ROTATIONAL_VELOCITY_CONSTANT = 0.2;

    public static Translation2d DISTANCE_FROM_ROBOT_CENTER = new Translation2d(0, 0);

    Translation3d TURRET_TO_CENTER_OF_ROBOT = new Translation3d(x, y, 0.32)
}
