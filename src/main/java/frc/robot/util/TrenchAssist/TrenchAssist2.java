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

public class TrenchAssist2 {

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



    public static ChassisSpeeds funky(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds speedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, drive.getYaw());
        ChassisSpeeds horizontalSpeeds = new ChassisSpeeds(speedsFieldRelative.vxMetersPerSecond, 0.0, speedsFieldRelative.omegaRadiansPerSecond);

        Logger.recordOutput("slideTranslation", new Translation2d(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond));

        var x = new ChassisSpeeds(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        var y = new Translation2d(x.vxMetersPerSecond, x.vyMetersPerSecond).rotateBy(drive.getYaw().unaryMinus());
        return new ChassisSpeeds(y.getX(), y.getY(), x.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
        double yawRadians = drive.getYaw().unaryMinus().getRadians();
        
        double fieldRelativeX = chassisSpeeds.vxMetersPerSecond * Math.cos(yawRadians) - chassisSpeeds.vyMetersPerSecond * Math.sin(yawRadians);
        double fieldRelativeY = chassisSpeeds.vxMetersPerSecond * Math.sin(yawRadians) + chassisSpeeds.vyMetersPerSecond * Math.cos(yawRadians);
        
        fieldRelativeY = 0;
        
        double neutralizedRobotX = fieldRelativeX * Math.cos(yawRadians) + fieldRelativeY * Math.sin(yawRadians);
        double neutralizedRobotY = -fieldRelativeX * Math.sin(yawRadians) + fieldRelativeY * Math.cos(yawRadians);

        var x = new Translation2d(neutralizedRobotX, neutralizedRobotY).rotateBy(drive.getYaw().unaryMinus());

        return new ChassisSpeeds(x.getX(), x.getY(), chassisSpeeds.omegaRadiansPerSecond);
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