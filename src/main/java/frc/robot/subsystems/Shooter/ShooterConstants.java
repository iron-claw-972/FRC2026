package frc.robot.subsystems.Shooter;

public class ShooterConstants {
    //TODO: find these values
    public static final double FEEDER_RUN_POWER = 0.3;
    public static final double SHOOTER_RUN_POWER = 0.3;

    // hood gear ratios
    public static final double HOOD_BOTTOM_GEAR_RATIO = 3.75 / 1.0;
    public static final double HOOD_TOP_GEAR_RATIO = 0.75 / 1.0;
    // total gear ratios
    public static final double HOOD_GEAR_RATIO = HOOD_BOTTOM_GEAR_RATIO * HOOD_TOP_GEAR_RATIO; // 2.8125:1

    // shooter gear ratios
    public static final double SHOOTER_SMALL_WHEEL_RATIO = 1.0 / 3.0;
    public static final double SHOOTER_BIG_WHEEL_RATIO = 1.5 / 1.0;
    // total gear ratios
    public static final double SHOOTER_GEAR_RATIO = SHOOTER_SMALL_WHEEL_RATIO * SHOOTER_BIG_WHEEL_RATIO; // 1:2
}
