package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static double MAX_ANGLE = 180;
    public static double MIN_ANGLE = -180;

    public static double MAX_VELOCITY = 1.0; // m/s
    public static double MAX_ACCELERATION = 5; // m/s^2

    public static double TURRET_WIDTH = Units.feetToMeters(1.0);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double ROTATIONAL_VELOCITY_CONSTANT = 0;
}
