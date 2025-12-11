package frc.robot.constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodConstants {
    public static final double HOOD_GEAR_RATIO = (3.75)*(24/18)*(163/10); // 81.5

    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138*2; // meters
    public static final double MOI = 0.0489969498; // kg*m^2 <-- We got this on Onshape

    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double MAX_VELOCITY = 5; // rad/s
    public static final double MAX_ACCELERATION = 30; // rad/s^2

    public static final double MAX_ANGLE = 51.48; // degrees
    public static final double MIN_ANGLE = 41.8; // degrees
    
    public static final double START_ANGLE = MAX_ANGLE; // degrees

    // Arena dimensions
    public static final double TARGET_HEIGHT = 1.6; // meters

    public static final Translation2d TRANSLATION_TARGET = new Translation2d(0, 0);
    public static final Rotation2d ROTATION_TARGET_ANGLE = new Rotation2d();
    public static final int tagNumber = 18;
    public static final Pose2d TARGET_POSITION = FieldConstants.APRIL_TAGS.get(tagNumber - 1).pose.toPose2d(); // meters
    // Other
    public static final double INITIAL_VELOCTIY = ShooterConstants.SHOOTER_VELOCITY; // meters per second

    // Testing purposes
    public static final double START_DISTANCE = 2; // meters

    // Calibration Purposes
    public static final double CURRENT_SPIKE_THRESHHOLD = 10.0; // amps
}
