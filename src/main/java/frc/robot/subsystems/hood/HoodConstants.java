package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HoodConstants {
    public static final double HOOD_GEAR_RATIO = 64;

    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138 * 2; // meters
    public static final double MOI = 0.0489969498; // kg*m^2 <-- We got this on Onshape

    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    // public static final double MAX_VELOCITY = 5; // rad/s
    public static final double MAX_VELOCITY = 0.30; // rad/s
    public static final double MAX_ACCELERATION = 30; // rad/s^2

    public static final double MAX_ANGLE = 82; // degrees
    public static final double MIN_ANGLE = 58.5; // degrees 

    public static final double START_ANGLE = 90-22.9; // degrees

    // Arena dimensions
    public static final double TARGET_HEIGHT = 2.44; // meters
    public static final double SHOOTER_HEIGHT = 0.51; // meters

    public static final Translation2d TRANSLATION_TARGET = new Translation2d(0, 0);
    public static final Rotation2d ROTATION_TARGET_ANGLE = new Rotation2d();
    // Other
    public static final double INITIAL_VELOCTIY = 14.9; // meters per second

    // Testing purposes
    public static final double START_DISTANCE = 2; // meters

    // Calibration Purposes
    public static final double CURRENT_SPIKE_THRESHHOLD = 10.0; // amps
}
