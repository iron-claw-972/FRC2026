package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static double MAX_ANGLE = 200;
    public static double MIN_ANGLE = -200;

    public static double MAX_VELOCITY = 600; // rad/s
    public static double MAX_ACCELERATION = 120.0; // rad/s^2

    // Not using this, but just in case
    public static double TURRET_WIDTH = Units.inchesToMeters(6.4);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double TURRET_TEETH_COUNT = 140.0; // the turret teeth count
    public static double TURRET_GEAR_RATIO = 36.81818182;
    public static int LEFT_ENCODER_TEETH = 15; // gear teeth
    public static int RIGHT_ENCODER_TEETH = 22; // read above
    public static int ENCODER_COUNT_TOTAL = 8192; // how many intervals it can have, like clicks on a clock chat gpt explained to me

    public static double LEFT_ENCODER_OFFSET = 9.52; // degrees 9.52 rot
    public static double RIGHT_ENCODER_OFFSET = 6.53; // degrees 6.53 rot

    // Turret is in center of robot, but make use of the height in shooter physics
    public static Translation3d DISTANCE_FROM_ROBOT_CENTER = new Translation3d(0,0, Units.inchesToMeters(22.172)); //meters

    public static double CRT_TOLERANCE = 0.01;
}