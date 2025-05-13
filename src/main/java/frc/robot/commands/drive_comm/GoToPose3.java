package frc.robot.commands.drive_comm;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Follows a trajectory to a pose, similar to PathPlanner
 * This is necessary because PathPlanner tends to crash into things and can't be controlled as easily
 * This algorithm is designed to approach the pose from a specific angle. Use a different one or set endDerivative to 0 if you want to approach from any angle
 * TODO: Test on a real robot. If it works as intended, delete GoToPose and GoToPose2 and rename this to GoToPose
 */
public class GoToPose3 extends Command {
    // If this is true, the robot's speed will be normalized to 1, which will make the same trajectory with any velocity
    // This does nothing if the speed is less than 1m/s, about 25%
    private static final boolean normalizeSpeeds = true;
    // The velocity, after normalization, will be multiplied by this
    private static final double velocityScale = 1;
    // This is similar to the tangent line on PathPlanner, where 0 means it will approach the point from any direction and 1 will weight it as much as a normalized velocity
    private static final double endDerivative = 3;
    // The offset (in radians) to add to the pose rotation to get the heading
    private static final double offset = 0;
    // The acceleration of the robot
    // This isn't perfectly accurate since it is along a curved path, but the main purpose of this is to make sure it comes to a stop without hitting anything
    // TODO: Increase speed and accel after it works at this speed
    private static final double accel = 3;
    // The max speed of the robot while following this path
    private static final double maxSpeed = 1.5;
    // The PID to correct the robot's motion perpendicular to the spline
    private static final PIDController correctionPID = new PIDController(10, 0, 0);
    // Tolerances
    private static final double translationTolerance = 0.015;
    private static final double thetaTolerance = Units.degreesToRadians(1.0);
    // Variables for the angular trapezoidal profile
    private static final double thetakP = 7.0;
    private static final double thetakD = 0.0;
    private static final double thetaMaxVelocity = 5.0;
    private static final double thetaMaxAcceleration = 5.0;
    // The maximum distance from the spline before the code will redraw it
    private static final double maxError = 0.3;
    // The distance to the end of the path where it will start using PID
    private static final double PIDRange = 0.05;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            thetakP, 0.0, thetakD, new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration), Constants.LOOP_TIME);
  
    private Drivetrain drive;
    private Supplier<Pose2d> poseSupplier;
    private Spline spline;
    private Pose2d pose;
    private Pose2d start;
    private double length;
    private double prevT;
    private double error;
    private double startSpeed;

    /**
     * Creates a new instance of this command
     * @param poseSupplier The supplier for the pose to align to
     * @param drive The robot's drivetrain
     */
    public GoToPose3(Supplier<Pose2d> poseSupplier, Drivetrain drive){
        this.poseSupplier = poseSupplier;
        this.drive = drive;
        addRequirements(drive);
        correctionPID.setSetpoint(0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Initializes the spline
     * This method is slightly inefficient, and can take about 1.5ms
     * This should not be enough to cause loop overruns, but don't initialize more than necessary
     */
    @Override
    public void initialize(){
        pose = poseSupplier.get();
        updateSpline();
        drive.setVisionEnabled(VisionConstants.ENABLED_GO_TO_POSE);
    }

    /**
     * Drives along the spline
     * This will find the nearest point and drive in the direction of the spline at that point
     * The speed is calculated using the distance to the start and end positions
     */
    @Override
    public void execute(){
        double t = getT();
        // If error is more than about a foot, redraw the spline
        if(Math.abs(error) > maxError){
            updateSpline();
            t = getT();
        }
        prevT = t;
        Translation2d velocity;
        Pose2d drivePose = drive.getPose();
        var optional = getSplinePoint(t);
        // If the pose doesn't exist, don't drive uncontrollably and try updating the spline
        if(optional.isEmpty()){
            // Using drive.stop() will override SwerveSetpointGenerator
            drive.setChassisSpeeds(new ChassisSpeeds(), false);
            updateSpline();
            return;
        }
        Pose2d pose = optional.get();
        // Calculate linear distance between the current point and the start and end
        // This estimation is close enough for estimating the distance traveled and distance left to travel near these points
        double l1 = pose.getTranslation().getDistance(start.getTranslation());
        double l2 = pose.getTranslation().getDistance(this.pose.getTranslation());
        // If the drivetrain is within 2 inches or has passed the setpoint, use PID
        if(l2 < PIDRange || t > 0.99999){
            velocity = new Translation2d(
                Math.abs(correctionPID.calculate(l2)),
                this.pose.getTranslation().minus(drivePose.getTranslation()).getAngle()
            );
        }else{
            // v=√(v0^2+2ax)
            // We want the next speed, so we then add or subtract at
            double speed = Math.min(Math.min(
                Math.sqrt(Math.pow(startSpeed, 2) + 2*accel*l1) + accel*Constants.LOOP_TIME,
                Math.max(Math.sqrt(2*accel*l2) - accel*Constants.LOOP_TIME, 0)
            ), maxSpeed);
            // Base velocity is the speed in the direction of travel
            velocity = new Translation2d(speed, pose.getRotation())
            // Correction factor is in the perpendicular direction
                .plus(new Translation2d(correctionPID.calculate(error), pose.getRotation().plus(new Rotation2d(Math.PI/2))));
        }
        // If a large correctin factor was added, constrain it to the max velocity
        if(velocity.getNorm() > maxSpeed){
            velocity = velocity.times(maxSpeed/velocity.getNorm());
        }
        double omega = thetaController.calculate(drivePose.getRotation().getRadians());
        drive.drive(velocity.getX(), velocity.getY(), omega, true, false);
    }

    /**
     * Stops the drivetrain and deletes the spline to save memory
     * @param interrupted This is ignored
     */
    @Override
    public void end(boolean interrupted){
        drive.stop();
        drive.setVisionEnabled(true);
        spline = null;
        pose = null;
        start = null;
    }

    /**
     * Returns whether the drivetrain's pose is within tolerance of the target
     * @return A boolean for whether the drivetrain is within tolerance
     */
    @Override
    public boolean isFinished(){
        Pose2d error = drive.getPose().relativeTo(pose);
        return error.getTranslation().getNorm() < translationTolerance && MathUtil.angleModulus(error.getRotation().getRadians()) < thetaTolerance;
    }

    /**
     * Recreates the spline from the current pose to the target pose
     */
    public void updateSpline(){
        start = drive.getPose();
        correctionPID.reset();
        error = 0;
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), start.getRotation());
        startSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        thetaController.setGoal(pose.getRotation().getRadians());
        thetaController.reset(start.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
        if(normalizeSpeeds){
            if(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 1){
                speeds = speeds.div(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
            }
        }
        speeds = speeds.times(velocityScale);
        // Modulus is unnecessary because this is only used for trig
        double angle = pose.getRotation().getRadians()+offset;
        spline = new CubicHermiteSpline(
            new double[]{start.getX(), speeds.vxMetersPerSecond},
            new double[]{pose.getX(), endDerivative*Math.cos(angle)},
            new double[]{start.getY(), speeds.vyMetersPerSecond},
            new double[]{pose.getY(), endDerivative*Math.sin(angle)}
        );
        prevT = 0;

        // Estimate length by finding the length of 50 small segments
        length = 0;
        PoseWithCurvature prevPoint = null;
        for(double t = 0; t < 1.01; t += 0.02){
            t = MathUtil.clamp(t, 0.000001, 0.99999);
            Optional<PoseWithCurvature> optional = spline.getPoint(t);
            if(!optional.isPresent())
                continue;
            PoseWithCurvature point = optional.get();
            if(prevPoint != null){
                // Approximated as the simple distance between points. This is the minimum possible distance.
                double straightDist = point.poseMeters.getTranslation().getDistance(prevPoint.poseMeters.getTranslation());
                // Use the curvature and change in angle to estimate it
                // Curvature = angle/distance, so distance = angle / average curvature
                // To avoid problems with two oppositee curvatures resulting in very large values, this averages the distance calculated with eacch curvature
                double theta = MathUtil.angleModulus(point.poseMeters.getRotation().minus(prevPoint.poseMeters.getRotation()).getRadians());
                double l1 = straightDist, l2 = straightDist;
                // Small curvatures can cause extremely large values
                // theta/curvature can be negative, but this means the path curves in the opposite direction of the angle change, so this calculation is meaningless
                if(Math.abs(point.curvatureRadPerMeter) > 0.25){
                    l1 = Math.max(straightDist, theta/point.curvatureRadPerMeter);
                }
                if(Math.abs(prevPoint.curvatureRadPerMeter) > 0.25){
                    l2 = Math.max(straightDist, theta/prevPoint.curvatureRadPerMeter);
                }
                length += (l1+l2)/2;
            }
            prevPoint = point;
        }
    }

    /**
     * Calculates the dreivetrain's position along the spline using repeated interpolation
     * This will also store the distance off the spline in the error instance variable
     * @return The position along the spline, from 0 to 1
     */
    public double getT(){
        Pose2d drivePose = drive.getPose();
        // The robot won't be behind its previous point
        double min = prevT;
        // Use 2*speed*dt as the maximum distance the robot can travel in a frame; this is intentionally more than the speed so the algorithm can catch up to the robot
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        double max = Math.min(prevT+2*Math.max(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 1)*Constants.LOOP_TIME/length, 1);
        var minOptional = getSplinePoint(min);
        var maxOptional = getSplinePoint(max);
        Pose2d minPose, maxPose;
        // If the poses exist, which they should, assign them normally
        // Otherwise, interpolate over the entire spline to prevent errors
        if(minOptional.isPresent()){
            minPose = minOptional.get();
        }else{
            min = 0;
            minPose = getSplinePoint(0).get();
        }
        if(maxOptional.isPresent()){
            maxPose = maxOptional.get();
        }else{
            max = 1;
            maxPose = getSplinePoint(1).get();
        }
        double t = 0;
        error = 0;
        for(int i = 0; i < 10; i++){
            double driveX = getX(drivePose, minPose, maxPose);
            // This guess for t is the average of the results of binary search and using the drivetrain's estimated position
            t = MathUtil.clamp((min+max)/4+(min+driveX/length)/2, min, max);
            var optional = getSplinePoint(t);
            // If the optional doesn't exist, it will never exist, so this should return early with the previous guess
            if(optional.isEmpty()){
                error = drivePose.relativeTo(minPose).getY();
                return min;
            }
            Pose2d p = optional.get();
            if(getX(p, minPose, maxPose) < driveX){
                min = t;
                minPose = p;
            }else{
                max = t;
                maxPose = p;
            }
            // If max and min are too close to each other, break
            if(max - min < 0.0001){
                break;
            }
        }
        error = drivePose.relativeTo(minPose).getY();
        return t;
    }

    /**
     * Gets the distance in meters of the pose along the axis from start to end
     * This is named getX because x is defined as the distance along the spline while y is perpendicular to it
     * @param pose The pose to find the position of
     * @param start The start pose; poses at this point will return 0
     * @param end The end pose; poses at this point will return the linear distance between start and end
     * @return The x position, in meters
     */
    public static double getX(Pose2d pose, Pose2d start, Pose2d end){
        Rotation2d angle = end.getTranslation().minus(start.getTranslation()).getAngle();
        return pose.getTranslation().minus(start.getTranslation()).rotateBy(angle.unaryMinus()).getX();
    }

    /**
     * Gets a point along the spline
     * Better than spline.getPoint() because it doesn't throw errors when t==0
     * @param t The position along the spline, from 0 to 1
     * @return The spline point as an optional
     */
    public Optional<Pose2d> getSplinePoint(double t){
        if(t<0.000001){
            ControlVector vector = spline.getInitialControlVector();
            return Optional.of(new Pose2d(vector.x[0], vector.y[0], new Rotation2d(vector.x[1], vector.y[1])));
        }else if(t > 0.999999){
            ControlVector vector = spline.getFinalControlVector();
            return Optional.of(new Pose2d(vector.x[0], vector.y[0], new Rotation2d(vector.x[1], vector.y[1])));
        }
        return spline.getPoint(t).map(p->p.poseMeters);
    }
}
