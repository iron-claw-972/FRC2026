package frc.robot.subsystems.hood;

public class HoodConstants {
    public static final double HOOD_GEAR_RATIO = 64;

    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138 * 2; // meters
    public static final double MOI = 0.0489969498; // kg*m^2 <-- We got this on Onshape

    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double MAX_VELOCITY = 25; // rad/s
    public static final double MAX_ACCELERATION = 160; // rad/s^2

    public static final double MAX_ANGLE = 82; // degrees
    public static final double MIN_ANGLE = 58.5; // degrees 

    public static final double FEEDFORWARD_KV = 0.12;

    public static final double NORMAL_CURRENT_LIMIT = 40.0; // A
    public static final double CALIBRATING_CURRENT_LIMIT = 10.0; //A
    public static final double CALIBRATION_CURRENT_THRESHOLD = 9.0; // A
}
