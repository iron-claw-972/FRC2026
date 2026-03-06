package frc.robot.util.TrenchAssist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrenchAssist {

    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds chassisSpeeds, PIDController pid) {
        // ChassisSpeeds speedsFieldRelative =
        // ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
        // drive.getYaw().unaryMinus());

        double distanceFromSlideLatitude;

        if (drive.getPose().getY() > (FieldConstants.FIELD_WIDTH / 2.0)) {
            distanceFromSlideLatitude = (drive.getPose().getY() - TrenchAssistConstants.SLIDE_LATITUDES[0]);
        } else {
            distanceFromSlideLatitude = (drive.getPose().getY() - TrenchAssistConstants.SLIDE_LATITUDES[1]);
        }

        Logger.recordOutput("slides",
                new Pose2d[] { new Pose2d(2.0, TrenchAssistConstants.SLIDE_LATITUDES[0], Rotation2d.kZero),
                        new Pose2d(2.0, TrenchAssistConstants.SLIDE_LATITUDES[1], Rotation2d.kZero) });

        double correctionVelocity = pid.calculate(distanceFromSlideLatitude, 0);

        if (Math.abs(distanceFromSlideLatitude) < Units.inchesToMeters(3)) {
            correctionVelocity = 0.0;
        }

        ChassisSpeeds horizontalSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, correctionVelocity,
                chassisSpeeds.omegaRadiansPerSecond);

        var y = new Translation2d(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond)
                .rotateBy(drive.getYaw());

        var z = ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(y.getX(), y.getY(), horizontalSpeeds.omegaRadiansPerSecond),
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
