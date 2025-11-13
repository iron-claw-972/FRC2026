package frc.robot.constants;

public class IntakeConstants {
    public static final double gravity = 9.81;
    public static final double arm_mass = 1.573263;
    public static final double center_of_mass = 0.1996;
    public static final double arm_length = center_of_mass * 2;
    public static final double momment_of_intertia = arm_mass * Math.pow(arm_length,2);

    public static final double kP=0.02;
    public static final double kI=0.0;
    public static final double kD=0.0;

    public static final double max_velocity = 1.0;
    public static final double max_acceleration = 2.0;

    public static final double gear_ratio = 38;

    public static final double robot_voltage = 12.0;

    public static final int flywheel_motor_id = 2;
    public static final int intake_motor_id = 3;
}
