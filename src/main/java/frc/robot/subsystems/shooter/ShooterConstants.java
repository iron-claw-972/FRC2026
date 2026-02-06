package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterConstants {
    //TODO: find these values
    public static final double FEEDER_RUN_POWER = 0.5; // meters per second??
    public static double SHOOTER_VELOCITY = 30.0; // meters per second
    public static final double SHOOTER_GEAR_RATIO = 36.0 / 24.0; // gear ratio from motors to shooter wheel
    // public static final double SHOOTER_LAUNCH_DIAMETER = 0.0762; // meters (3 inches)
    public static final double SHOOTER_LAUNCH_DIAMETER = 0.1016; // meters (4 inches) I think this is right.
    public static final double SHOOTER_RUN_POWER = 0.3;
    public static final double SENSOR_DISTANNCE_THRESHOLD = 0.150; // meters
    public static final double SENSOR_AMBIENCE_THRESHOLD = 0.100; // meters

    // in m/s
    public static final double EXIT_VELOCITY_TOLERANCE = 1.0;

    // for bang bang
    public static final double TORQUE_CURRENT_CONTROL_TOLERANCE = 10; // velocity (rotations per second)

}
// 8 velcocity is too little
// 16 is too much