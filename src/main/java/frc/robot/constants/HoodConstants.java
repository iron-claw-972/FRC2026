package frc.robot.constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class HoodConstants {
    //TODO: update all of these constants
    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138*2; // meters
    public static final double MOI = 0.0261057394; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double MAX_VELOCITY = 21; // rad/s
    public static final double MAX_ACCELERATION = 120; // rad/s^2

    public static final double START_ANGLE = 0;

    // Arena dimensions
    public static final double TARGET_HEIGHT = 1.6; // meters

    public static final Translation2d TRANSLATION_TARGET = new Translation2d(0, 0);
    public static final Rotation2d ROTATION_TARGET_ANGLE = new Rotation2d();
    public static final Pose2d TARGET_POSITION = new Pose2d(TRANSLATION_TARGET, ROTATION_TARGET_ANGLE); // meters
    // Other
    public static final double INITIAL_VELOCTIY = 8; // meters per second

    // Testing purposes
    public static final double START_DISTANCE = 2; // meters
}
