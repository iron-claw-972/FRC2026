package frc.robot.util.TrenchAssist;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DriverAssist;
import frc.robot.util.TrenchAssist.TrenchAssistConstants;

public class TrenchAssist {

    // public void execute() {
    //     trenchAlign = SmartDashboard.getBoolean("trench aligning", trenchAlign);
    //     trenchAssist = SmartDashboard.getBoolean("trench aligning", trenchAssist);

    //     SmartDashboard.putBoolean("trench aligning", trenchAlign);
    //     SmartDashboard.putBoolean("trench assisting", trenchAssist);

    //     double forwardTranslation = driver.getForwardTranslation();
    //     double sideTranslation = driver.getSideTranslation();
    //     double rotation = -driver.getRotation();

    //     double slowFactor = driver.getIsSlowMode() ? DriveConstants.SLOW_DRIVE_FACTOR : 1;

    //     forwardTranslation *= slowFactor;
    //     sideTranslation *= slowFactor;
    //     rotation *= driver.getIsSlowMode() ? DriveConstants.SLOW_ROT_FACTOR : 1;

    //     int allianceReversal = Robot.getAlliance() == Alliance.Red ? 1 : -1;
    //     forwardTranslation *= allianceReversal;
    //     sideTranslation *= allianceReversal;

    //     ChassisSpeeds driverInput = new ChassisSpeeds(forwardTranslation, sideTranslation, rotation);
    //     ChassisSpeeds corrected = DriverAssist.calculate(drive, driverInput, drive.getDesiredPose(), true);

    //     if (trenchAlign) {
    //         for (Rectangle2d rectangle : TrenchAssistConstants.ALIGN_ZONES) {
    //             if (rectangle.contains(drive.getPose().getTranslation())) {
    //                 drive.setIsAlign(true);

    //                 if (drive.getPose().getRotation().getDegrees() > 90
    //                         && drive.getPose().getRotation().getDegrees() < 270) {
    //                     drive.setAlignAngle(0.0);
    //                 } else if (drive.getPose().getRotation().getDegrees() > 270
    //                         && drive.getPose().getRotation().getDegrees() < 90) {
    //                     drive.setAlignAngle(Units.degreesToRadians(180));
    //                 }
    //             } else {
    //                 drive.setIsAlign(false);
    //             }
    //         }
    //     }

    //     if (trenchAssist) {
    //         Translation2d calculated = calculateCorrection(TrenchAssistConstants.OBSTACLES);
    //         ChassisSpeeds assisted = new ChassisSpeeds(corrected.vxMetersPerSecond + calculated.getX(),
    //                 corrected.vyMetersPerSecond + calculated.getY(), corrected.omegaRadiansPerSecond);

    //         drive(assisted);
    //     } else {
    //         drive(corrected);
    //     }

    // }

    /**
     * 
     * @param rectangle the rectangle that the ray should check against
     * @param point     the origin point of the ray
     * @param velocity  vector of the ray, magnitude is speed
     * @param time      how far into the future to check
     * @return returns Optional full if intersects rectangle within time param into
     *         future, time/100
     *         second resolution for raymarching, returns distance that it hits it
     *         at
     */
    public static Optional<Double> rayCast(Rectangle2d rectangle, Translation2d point, Translation2d velocity,
            double time) {
        // double distance = velocity.getNorm();
        // Translation2d normalized = new Translation2d(velocity.getX() / distance,
        // velocity.getY() / distance);

        for (int i = 0; i <= 100; i++) {
            double t = (i * time) / 100.0; // seconds into the future
            Translation2d ray = velocity.times(t).plus(point);
            if (rectangle.contains(ray)) {
                return Optional.of(t);
            }
        }

        return Optional.empty();
    }

    static Translation2d calculateCorrection(Rectangle2d[] rectangles, Drivetrain drive, ChassisSpeeds predictedSpeeds) {
        Pose2d pose = drive.getPose();

        Translation2d velocityRobotRelative = new Translation2d(predictedSpeeds.vxMetersPerSecond,
                predictedSpeeds.vyMetersPerSecond);

        Translation2d velocityFieldRelative = velocityRobotRelative.rotateBy(drive.getYaw());

        double halfRobotWidth = 0.5 * DriveConstants.ROBOT_WIDTH_WITH_BUMPERS;

        Translation2d[] robotCorners = new Translation2d[] {
                pose.transformBy(new Transform2d(new Translation2d(halfRobotWidth, 0), new Rotation2d(0.0)))
                        .getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, halfRobotWidth), new Rotation2d(0.0)))
                        .getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(-halfRobotWidth, 0), new Rotation2d(0.0)))
                        .getTranslation(),
                pose.transformBy(new Transform2d(new Translation2d(0, -halfRobotWidth), new Rotation2d(0.0)))
                        .getTranslation(),
        };

        for (Translation2d corner : robotCorners) {
            for (Rectangle2d rectangle : rectangles) {
                Optional<Double> distanceOptional = rayCast(rectangle, corner, velocityFieldRelative, 1.0);
                if (distanceOptional.isPresent()) {
                    double timeToCollision = distanceOptional.get(); // seconds
                    double timeWindow = 1.0; //secs
                    double scale = Math.max(0.0, 1.0 - (timeToCollision / timeWindow));


                    if (drive.getPose().getY() > rectangle.getCenter().getY() + (rectangle.getYWidth() / 2)) {
                        // above rectangle: push +90 deg
                        return velocityFieldRelative.rotateBy(new Rotation2d(Units.degreesToRadians(90))).times(scale);
                    } else if (drive.getPose().getY() <= rectangle.getCenter().getY() - (rectangle.getYWidth() / 2)) {
                        // below rectangle push -90 deg
                        return velocityFieldRelative.rotateBy(new Rotation2d(Units.degreesToRadians(-90))).times(scale);
                    }

                    // emergency fallback
                    if (rayCast(rectangle, corner, velocityFieldRelative, 0.2).isPresent()) {
                        return velocityFieldRelative.unaryMinus();
                    }
                }
            }
        }

        return new Translation2d(0, 0);

    }


    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
        Translation2d correctionFieldRelative = calculateCorrection(TrenchAssistConstants.OBSTACLES, drive, chassisSpeeds);

        Logger.recordOutput("TrenchCorrectionFieldRelative", correctionFieldRelative);

        ChassisSpeeds correctionRobot = convertToChassisSpeedsRobotRelative(correctionFieldRelative, drive);

        double vx = chassisSpeeds.vxMetersPerSecond + correctionRobot.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond + correctionRobot.vyMetersPerSecond;

        return new ChassisSpeeds(vx, vy, chassisSpeeds.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds convertToChassisSpeedsRobotRelative(Translation2d translation, Drivetrain drive){
        Rotation2d yaw = drive.getYaw();
        Translation2d robotRelative = translation.rotateBy(yaw);
        return new ChassisSpeeds(robotRelative.getX(), robotRelative.getY(), 0.0);

    }

    public static Translation2d convertToTranslation2dFieldRelative(ChassisSpeeds speeds, Drivetrain drive){
        Rotation2d yaw = drive.getYaw();
        Translation2d robotTranslation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return robotTranslation.rotateBy(yaw.times(-1));

    }
}