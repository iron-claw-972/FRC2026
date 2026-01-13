package frc.robot.commands.drive_comm;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
     * @return true if ray intersects rectangle within time param into future, time/100
     *         second resolution for raymarching
     */
    public static boolean rayCast(Rectangle2d rectangle, Translation2d point, Translation2d velocity, double time) {
        // double distance = velocity.getNorm();
        // Translation2d normalized = new Translation2d(velocity.getX() / distance,
        // velocity.getY() / distance);

        for (int i = 0; i <= 100; i++) {
            Translation2d ray = velocity.times((i * time) / 100.0).plus(point);
            if (rectangle.contains(ray)) {
                return true;
            }
        }

        return false;
    }

    Translation2d calculateCorrection(Rectangle2d[] rectangles) {
        Pose2d pose = drive.getPose();
        Translation2d velocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
                drive.getChassisSpeeds().vyMetersPerSecond);

        Translation2d[] robotCorners = new Translation2d[] {
                pose.transformBy(new Transform2d(new Translation2d(1, 0), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, 1), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(-1, 0), new Rotation2d(0.0))).getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, -1), new Rotation2d(0.0))).getTranslation(),
        }; // TODO add actual corner locations

        for (Translation2d corner : robotCorners) {
            for (Rectangle2d rectangle : rectangles) {
                if (rayCast(rectangle, corner, velocity, 1.0)) {
                    // correction to push perpendicular to rectangle
                    if (drive.getPose().getY() > rectangle.getCenter().getY() + (rectangle.getYWidth() / 2)) {
                        // above rectangle
                        return new Translation2d(0, 1).times(0.5);
                    } else if (drive.getPose().getY() <= rectangle.getCenter().getY() - (rectangle.getYWidth() / 2)) {
                        // below rectangle
                        return new Translation2d(0, -1).times(0.5);

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

                    if (rayCast(rectangle, corner, velocity, 0.2)) {
                        return velocity.unaryMinus(); // fallback if uh oh
                        // alex won't like robot stopping suddenly, so only if about to crash
                    }
                }
            }
        }

        return new Translation2d(0, 0);

    }

}
