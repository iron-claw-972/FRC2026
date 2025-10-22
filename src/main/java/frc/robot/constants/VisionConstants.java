package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Container class for vision constants.
 */
public class VisionConstants {
    /**
     * If April tag vision is enabled on the robot
     */
    public static final boolean ENABLED = false;

    /**
     * If object detection should be enabled
     */
    public static final boolean OBJECT_DETECTION_ENABLED = false;

    /** If odometry should be updated using vision during auto */
    public static final boolean ENABLED_AUTO = true;

    /**
     * If odometry should be updated using vision while running the GoToPose,
     * GoToPosePID, and DriveToPose commands in teleop
     */
    public static final boolean ENABLED_GO_TO_POSE = true;

    /** If vision should be simulated */
    public static final boolean ENABLED_SIM = false;

    /** If vision should only return values if it can see 2 good targets */
    public static final boolean ONLY_USE_2_TAGS = false;

    /** PoseStrategy to use in pose estimation */
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /** Fallback PoseStrategy if MultiTag doesn't work */
    public static final PoseStrategy MULTITAG_FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

    /**
     * Any April tags we always want to ignore. To ignore a tag, put its id in this
     * array.
     */
    public static final int[] TAGS_TO_IGNORE = {12, 13, 16, 1, 2, 3};

    /**
     * If multiple cameras return different poses, they will be ignored if the
     * difference between them is greater than this
     */
    public static final double MAX_POSE_DIFFERENCE = 0.2;

    /**
     * The maximum distance to the tag to use
     */
    public static final double MAX_DISTANCE = 2;

    /** If vision should use manual calculations */
    public static final boolean USE_MANUAL_CALCULATIONS = true;

    // <ol start="0"> did not work
    /**
     * Which version of driver assist to use. This would be an enum, except there is
     * no short and descriptive name for all of these.
     * <p>
     * The options are:
     * <p>
     * 0: Disable driver assist
     * <p>
     * 1: Completely remove the component of the driver's input that is not toward
     * the object
     * <p>
     * 2: Interpolate between the next achievable driver speed and a speed
     * calculated using trapezoid profiles
     * <p>
     * 3-5: Add a speed perpendicular to the driver input; there are 3 similar but
     * different calculations for this
     */
    public static final int DRIVER_ASSIST_MODE = 5;

    /**
     * The number to multiply the distance to the April tag by.
     * <p>
     * Only affects manual calculations.
     * <p>
     * To find this, set it to 1 and measure the actual distance and the calculated
     * distance.
     * <p>
     * This should not be needed, and it is only here because it improved the
     * accuracy of vision in the 2023 fall semester
     */
    public static final double DISTANCE_SCALE = 1;

    /**
     * The standard deviations to use for vision
     */
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(
            0.3, // x in meters (default=0.9)
            0.3, // y in meters (default=0.9)
            0.9 // heading in radians. The gyroscope is very accurate, so as long as it is reset
                // correctly it is unnecessary to correct it with vision
    );

    /**
     * The standard deviations to use for vision when the wheels slip
     */
    public static final Matrix<N3, N1> VISION_STD_DEVS_2 = VecBuilder.fill(
            0.01, // x in meters (default=0.9)
            0.01, // y in meters (default=0.9)
            0.9 // heading in radians. The gyroscope is very accurate, so as long as it is reset
                // correctly it is unnecessary to correct it with vision
    );

    /**
     * The highest ambiguity to use. Ambiguities higher than this will be ignored.
     * <p>
     * Only affects calculations using PhotonVision, not manual calculations.
     */
    public static final double HIGHEST_AMBIGUITY = 0.01;

    public static final int MAX_EMPTY_TICKS = 10;

    /**
     * The camera poses
     * <p>
     * Everything is in meters and radians
     * <p>
     * 0 for all numbers is center of the robot, on the ground, looking straight
     * toward the front
     * <p>
     * + X: Front of Robot
     * <p>
     * + Y: Left of Robot
     * <p>
     * + Z: Top of Robot
     * <p>
     * + Pitch: Down
     * <p>
     * + Yaw: Counterclockwise
     */
    public static final ArrayList<Pair<String, Transform3d>> APRIL_TAG_CAMERAS = new ArrayList<Pair<String, Transform3d>>(
            List.of(
                new Pair<String, Transform3d>(
                        "CameraFront",
                        new Transform3d(
                                new Translation3d(Units.inchesToMeters(10.485), Units.inchesToMeters(10.217),
                                        Units.inchesToMeters(11.012)),
                                new Rotation3d(0, Units.degreesToRadians(-11),
                                        Math.PI/2 + Units.degreesToRadians(20)))),
                new Pair<String, Transform3d>(
                        "CameraBack",
                        new Transform3d(
                                new Translation3d(Units.inchesToMeters(-9.538), Units.inchesToMeters(7.474),
                                        Units.inchesToMeters(8.719)),
                                new Rotation3d(0, Units.degreesToRadians(-19.5),
                                        Math.PI/2-Units.degreesToRadians(25))))));

    /**
     * The transformations from the robot to object detection cameras
     */
    public static final ArrayList<Transform3d> OBJECT_DETECTION_CAMERAS = new ArrayList<>(List.of(
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(24)),
                    new Rotation3d(0, Units.degreesToRadians(20), 0))));
}