package frc.robot.commands.drive_comm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.TrenchAssist.TrenchAssist;
import frc.robot.util.TrenchAssist.TrenchAssistConstants;
import frc.robot.util.Vision.DriverAssist;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends Command {
    protected final Drivetrain swerve;
    protected final BaseDriverConfig driver;

    public DefaultDriveCommand(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.swerve = swerve;
        this.driver = driver;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setStateDeadband(true);
    }

    private boolean trenchAlign = false;
    private boolean trenchAssist = true;

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
        ChassisSpeeds corrected = DriverAssist.calculate(swerve, driverInput, swerve.getDesiredPose(), true);

        Logger.recordOutput("TrenchAlign", swerve.getTrenchAlign());
        Logger.recordOutput("AlignZones", TrenchAssistConstants.ALIGN_ZONES);
        if (swerve.getTrenchAlign()) {
            boolean inZone = false;
            for (Rectangle2d rectangle : TrenchAssistConstants.ALIGN_ZONES) {
                if (rectangle.contains(swerve.getPose().getTranslation())) {
                    inZone = true;
                }
            }

            if (inZone) {
                Logger.recordOutput("InAlignZone", true);
                swerve.setIsAlign(true);

                double yawDegrees = swerve.getYaw().getDegrees();
                double snappedDeg = Math.round(yawDegrees / 90.0) * 90.0;
                swerve.setAlignAngle(Units.degreesToRadians(snappedDeg));
            } else {
                Logger.recordOutput("InAlignZone", false);
                swerve.setIsAlign(false);
            }
        } else {
            swerve.setIsAlign(false);
        }

        Logger.recordOutput("TrenchAssist", swerve.getTrenchAssist());
        if (swerve.getTrenchAssist()) {
            drive(TrenchAssist.calculate(swerve, corrected));
        } else {
            drive(corrected);
        }

        Logger.recordOutput("isAlign", swerve.getIsAlign());
        Logger.recordOutput("alignAngle", swerve.getAlignAngle());
    }

    /**
     * Drives the robot
     * 
     * @param speeds The ChassisSpeeds to drive at
     */
    protected void drive(ChassisSpeeds speeds) {
        // If the driver is pressing the align button or a command set the drivetrain to
        // align, then align to speaker
        if (driver.getIsAlign() || swerve.getIsAlign()) {
            swerve.driveHeading(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    swerve.getAlignAngle(),
                    true);
        } else {
            swerve.drive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    true,
                    false);
        }
    }
}