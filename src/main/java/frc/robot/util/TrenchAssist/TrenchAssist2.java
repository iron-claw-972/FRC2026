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

    public static ChassisSpeeds calculate(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds speedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, drive.getYaw().unaryMinus()); //this singular unary minus is best we can get
        ChassisSpeeds horizontalSpeeds = new ChassisSpeeds(speedsFieldRelative.vxMetersPerSecond, 0.0, speedsFieldRelative.omegaRadiansPerSecond);

        Logger.recordOutput("slideTranslation", new Translation2d(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond));

        var x = new ChassisSpeeds(horizontalSpeeds.vxMetersPerSecond, horizontalSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        var y = new Translation2d(x.vxMetersPerSecond, x.vyMetersPerSecond).rotateBy(drive.getYaw());
        return new ChassisSpeeds(y.getX(), y.getY(), x.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds funky(Drivetrain drive, ChassisSpeeds chassisSpeeds) {
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