package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
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

    Translation3d TURRET_TO_CENTER_OF_ROBOT = new Translation3d(Units.inchesToMeters(4.875), -Units.inchesToMeters(26.5/2 - 11.375), 0.32);
    Translation3d CAMERA_TO_CENTER_OF_TURRET = new Translation3d(0.177, Units.inchesToMeters(1.0/8.0), 0.325);
    Rotation3d CAMERA_ROTATION = new Rotation3d(Units.degreesToRadians(-10), Units.degreesToRadians(-13), Units.degreesToRadians(-25));
}
