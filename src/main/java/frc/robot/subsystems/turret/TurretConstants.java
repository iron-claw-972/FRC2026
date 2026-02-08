package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
    public static double MAX_ANGLE = 180;
    public static double MIN_ANGLE = -180;

    public static double MAX_VELOCITY = 10000000; // m/s
    public static double MAX_ACCELERATION = 10000000; // m/s^2

    public static double TURRET_WIDTH = Units.feetToMeters(1.0);
    public static double TURRET_RADIUS = TURRET_WIDTH / 2;

    public static double ROTATIONAL_VELOCITY_CONSTANT = 0.2;

    public static double TURRET_GEAR_RATIO = 140; // the turret teeth count
    public static double LEFT_ENCODER_RATIO = 70/11; // read right description
    public static double RIGHT_ENCODER_RATIO = 28/3; // The amount of times this encoder turns for every time the turret turns
    public static double ENCODER_COUNT_TOTAL = 8192; // how many intervals it can have, like clicks on a clock chat gpt explained to me

    public static double LEFT_ENCODER_OFFSET = 0; // degrees
    public static double RIGHT_ENCODER_OFFSET = 0; // degrees

    public static Translation2d DISTANCE_FROM_ROBOT_CENTER = new Translation2d(0,0);

}