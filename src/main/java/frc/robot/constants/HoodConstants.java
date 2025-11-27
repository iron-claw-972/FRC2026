package frc.robot.constants;

public class HoodConstants {
    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138*2; // meters
    public static final double MOI = 0.0261057394; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double gravity = 9.8;

    public static final double MAX_VELOCITY = 10; // rad/s
    public static final double MAX_ACCELERATION = 120; // rad/s^2

    public static final double START_ANGLE = 0;

    public static final int motor_id = 1;

    private static final double hood_gear_ratio = 38.0;
    private static final double hood_ratio = 1.0;

    public static final double gear_ratio = hood_ratio/hood_gear_ratio;

    public static final double LOOP_TIME = 0.02;
}
