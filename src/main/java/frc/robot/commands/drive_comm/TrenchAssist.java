package frc.robot.commands.drive_comm;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.TrenchAssistConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DriverAssist;

public class TrenchAssist extends Command {

    protected final Drivetrain drive;
    protected final BaseDriverConfig driver;

    public TrenchAssist(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.drive = swerve;
        this.driver = driver;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        drive.setStateDeadband(true);
    }

    @Override
    public void execute() {
        double forwardTranslation = driver.getForwardTranslation();
        double sideTranslation = driver.getSideTranslation();
        double rotation = -driver.getRotation();

        double slowFactor = driver.getIsSlowMode() ? DriveConstants.SLOW_DRIVE_FACTOR : 1;

        forwardTranslation *= slowFactor;
        sideTranslation *= slowFactor;
        rotation *= driver.getIsSlowMode() ? DriveConstants.SLOW_ROT_FACTOR : 1;

        int allianceReversal = Robot.getAlliance() == Alliance.Red ? 1 : -1;
        forwardTranslation *= allianceReversal;
        sideTranslation *= allianceReversal;

        ChassisSpeeds driverInput = new ChassisSpeeds(forwardTranslation, sideTranslation, rotation);
        ChassisSpeeds corrected = DriverAssist.calculate(drive, driverInput, drive.getDesiredPose(), true);

        Translation2d calculated = calculateCorrection(TrenchAssistConstants.OBSTACLES);
        ChassisSpeeds assisted = new ChassisSpeeds(corrected.vxMetersPerSecond + calculated.getX(), 
            corrected.vyMetersPerSecond + calculated.getY(), corrected.omegaRadiansPerSecond);

        drive(assisted);

    }

    /**
     * Drives the robot
     * 
     * @param speeds The ChassisSpeeds to drive at
     */
    protected void drive(ChassisSpeeds speeds) {
        // If the driver is pressing the align button or a command set the drivetrain to
        // align, then align to speaker
        if (driver.getIsAlign() || drive.getIsAlign()) {
            drive.driveHeading(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    drive.getAlignAngle(),
                    true);
        } else {
            drive.drive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    true,
                    false);
        }
    }

    /**
     * 
     * @param rectangle the rectangle that the ray should check against
     * @param point     the origin point of the ray
     * @param velocity  vector of the ray, magnitude is speed
     * @param time      how far into the future to check
     * @return returns Optional full if intersects rectangle within time param into future, time/100
     *         second resolution for raymarching, returns distance that it hits it at
     */
    public static Optional<Double> rayCast(Rectangle2d rectangle, Translation2d point, Translation2d velocity, double time) {
        // double distance = velocity.getNorm();
        // Translation2d normalized = new Translation2d(velocity.getX() / distance,
        // velocity.getY() / distance);

        for (int i = 0; i <= 100; i++) {
            Translation2d ray = velocity.times((i * time) / 100.0).plus(point);
            if (rectangle.contains(ray)) {
                return Optional.of((i / 100.0) * velocity.getNorm());
            }
        }

        return Optional.empty();
    }

    Translation2d calculateCorrection(Rectangle2d[] rectangles) {
        Pose2d pose = drive.getPose();
        Translation2d velocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
                drive.getChassisSpeeds().vyMetersPerSecond);

        double robotCornerRadius = 0.5 * Math.sqrt(2 * Math.pow(DriveConstants.ROBOT_WIDTH_WITH_BUMPERS, 2));
        double halfRobotWidth = 0.5 * DriveConstants.ROBOT_WIDTH_WITH_BUMPERS;

        Translation2d[] robotCorners = new Translation2d[] {
                pose.transformBy(new Transform2d(new Translation2d(halfRobotWidth, 0), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, halfRobotWidth), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(-halfRobotWidth, 0), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, -halfRobotWidth), new Rotation2d(0.0))).getTranslation(),
        };

        for (Translation2d corner : robotCorners) {
            for (Rectangle2d rectangle : rectangles) {
                Optional<Double> distanceOptional = rayCast(rectangle, corner, velocity, 1.0);
                if (distanceOptional.isPresent()) {
                    // correction to push perpendicular to rectangle
                    if (drive.getPose().getY() > rectangle.getCenter().getY() + (rectangle.getYWidth() / 2)) {
                        // above rectangle
                        return velocity.rotateBy(new Rotation2d(Units.degreesToRadians(90))).times(distanceOptional.get() / velocity.getNorm());
                    } else if (drive.getPose().getY() <= rectangle.getCenter().getY() - (rectangle.getYWidth() / 2)) {
                        // below rectangle
                        return velocity.rotateBy(new Rotation2d(Units.degreesToRadians(-90))).times(distanceOptional.get() / velocity.getNorm()); // meters / (meters / second) -> (meters * seconds) / meters -> seconds, between 0 and 1 because time value to raycast is 1

                        // Are these last two necessary?
                        
                    // } else if (drive.getPose().getX() > rectangle.getCenter().getX() +
                    // (rectangle.getXWidth() / 2)){
                    // //right of rectangle
                    // return new Translation2d(1, 0).times(0.5);
                    // } else if (drive.getPose().getX() < rectangle.getCenter().getX() +
                    // (rectangle.getXWidth() / 2)){
                    // //left of rectangle
                    // return new Translation2d(-1, 0).times(0.5);
                    }

                    if (rayCast(rectangle, corner, velocity, 0.2).isPresent()) {
                        return velocity.unaryMinus(); // fallback if uh oh
                        // alex won't like robot stopping suddenly, so only if about to crash
                    }
                }
            }
        }

        return new Translation2d(0, 0);

    }

}
