package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static double MAX_ANGLE = 200;
    public static double MIN_ANGLE = -200;

    public static double MAX_VELOCITY = 10000000; // m/s
    public static double MAX_ACCELERATION = 10000000; // m/s^2

    public static double TURRET_WIDTH = Units.inchesToMeters(6.4);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double TURRET_TEETH_COUNT = 140.0; // the turret teeth count
    public static double TURRET_GEAR_RATIO = 36.81818182;
    public static double LEFT_ENCODER_RATIO = 70.0/11.0; // read right description
    public static double RIGHT_ENCODER_RATIO = 28.0/3.0; // The amount of times this encoder turns for every time the turret turns
    public static double ENCODER_COUNT_TOTAL = 8192.0; // how many intervals it can have, like clicks on a clock chat gpt explained to me

    public static double LEFT_ENCODER_OFFSET = Units.rotationsToDegrees(9.52); // degrees 9.52 rot
    public static double RIGHT_ENCODER_OFFSET = Units.rotationsToDegrees(6.53); // degrees 6.53 rot

    public static Translation3d DISTANCE_FROM_ROBOT_CENTER = new Translation3d(0,0, Units.inchesToMeters(22.172)); //meters

}