package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DriverAssist;

/**
 * faces forward, clone of defalut drive command
 */
public class FaceForward extends Command {
    private Drivetrain drive;
    private BaseDriverConfig driver;

    private final double FACEFORWARD_DEADZONE = 0.1;

    public FaceForward(Drivetrain drive, BaseDriverConfig driver) {
        this.drive = drive;
        this.driver = driver;
    }

    @Override
    public void initialize() {
        drive.setIsAlign(true);
        driver.getRawHeadingAngle();
    }

    @Override
    public void execute() {
        Translation2d velocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d desiredAngle = new Rotation2d(velocity.getX(), velocity.getY());

        if (driver.getRawHeadingMagnitude() > FACEFORWARD_DEADZONE) {
            drive.setAlignAngle(driver.getRawHeadingAngle());
        } else {
            drive.setAlignAngle(desiredAngle.getRadians());
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setIsAlign(false);
    }
}
