package frc.robot.commands.drive_comm;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
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

        drive(corrected);
    }

    /**
     * Drives the robot
     * @param speeds The ChassisSpeeds to drive at
     */
    protected void drive(ChassisSpeeds speeds){
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

    public static boolean rayCast(Rectangle2d rectangle, Translation2d point, Translation2d velocity){
        //double distance = velocity.getNorm();
        //Translation2d normalized = new Translation2d(velocity.getX() / distance, velocity.getY() / distance);

        for (int i = 0; i <= 100 ; i++){
            Translation2d ray = velocity.times(i / 100.0).plus(point);
            if (rectangle.contains(ray)){
                return true;
            }
        }

        return false;
    }


    Translation2d calculateCorrection(Rectangle2d[] rectangles){
        
    }

}
