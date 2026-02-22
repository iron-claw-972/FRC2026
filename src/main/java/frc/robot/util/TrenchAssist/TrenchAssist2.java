package frc.robot.util.TrenchAssist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrenchAssist2 {

    private static final double SCALE = 1.0;

    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
        // ChassisSpeeds speedsFieldRelative =
        // ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
        // drive.getYaw().unaryMinus()); //this singular unary minus is best we can get

        double distanceFromSlideLatitude;

        if (drive.getPose().getY() > (8.07 / 2.0)) {
            distanceFromSlideLatitude = (drive.getPose().getY() - TrenchAssistConstants.SLIDE_LATITUDES[0]);
        } else {
            distanceFromSlideLatitude = (drive.getPose().getY() - TrenchAssistConstants.SLIDE_LATITUDES[1]);
        }

        var impulse = -distanceFromSlideLatitude * SCALE;

        ChassisSpeeds horizontalSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, impulse,
                chassisSpeeds.omegaRadiansPerSecond);

        var x = new ChassisSpeeds(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
        var y = new Translation2d(x.vxMetersPerSecond, x.vyMetersPerSecond).rotateBy(drive.getYaw());

        var z = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(y.getX(), y.getY(), x.omegaRadiansPerSecond),
                drive.getYaw());

        return z;

    }

    public static ChassisSpeeds convertToChassisSpeedsRobotRelative(Translation2d translation, Drivetrain drive) {
        Rotation2d yaw = drive.getYaw();
        Translation2d robotRelative = translation.rotateBy(yaw);
        return new ChassisSpeeds(robotRelative.getX(), robotRelative.getY(), 0.0);

    }

    public static Translation2d convertToTranslation2dFieldRelative(ChassisSpeeds speeds, Drivetrain drive) {
        Rotation2d yaw = drive.getYaw();
        Translation2d robotTranslation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return robotTranslation.rotateBy(yaw.times(-1));

    }
}