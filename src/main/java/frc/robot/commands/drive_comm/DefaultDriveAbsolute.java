package frc.robot.commands.drive_comm;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DriverAssist;


/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveAbsolute extends Command {
    private final double thetaMaxVelocity = 5.0; 
    private final double thetaMaxAcceleration = 5.0; 
    private final double thetakP = 5.0;
    private final double thetaTolerance = Units.degreesToRadians(1.0); 

    private final Drivetrain swerve;
    private final BaseDriverConfig driver;
    private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP, 0.0, 0.0, new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration), Constants.LOOP_TIME);
    private double thetaErrorAbs = 0.0;

    private Supplier<Pose2d> robot;

    public DefaultDriveAbsolute(
            Drivetrain swerve,
            BaseDriverConfig driver) {
        this.swerve = swerve;
        this.driver = driver;

        robot = swerve::getPose;

        thetaController.setTolerance(thetaTolerance);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), currentPose.getRotation());

        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);

        swerve.setStateDeadband(true);
    }

    @Override
    public void execute() {
        double forwardTranslation = driver.getForwardTranslation();
        double sideTranslation = driver.getSideTranslation();

        double desiredHeading = driver.getHeading(); 
        double currentHeading = swerve.getYaw().getRadians();
         
        // Calculate theta speed 
        double thetaVelocity =
            thetaController.getSetpoint().velocity
            + thetaController.calculate(
                currentHeading, desiredHeading);
        thetaErrorAbs = Math.abs(currentHeading - desiredHeading);
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        double slowFactor = driver.getIsSlowMode() ? DriveConstants.SLOW_DRIVE_FACTOR : 1;

        forwardTranslation *= slowFactor;
        sideTranslation *= slowFactor;
        thetaVelocity *= driver.getIsSlowMode() ? DriveConstants.SLOW_ROT_FACTOR : 1;

        int allianceReversal = Robot.getAlliance() == Alliance.Red ? 1 : -1;
        forwardTranslation *= allianceReversal;
        sideTranslation *= allianceReversal;

        ChassisSpeeds driverInput = new ChassisSpeeds(forwardTranslation, sideTranslation, thetaVelocity);
        ChassisSpeeds corrected = DriverAssist.calculate(swerve, driverInput, swerve.getDesiredPose(), true);

        // If the driver is pressing the align button or a command set the drivetrain to
        // align, then align to speaker
        if (driver.getIsAlign() || swerve.getIsAlign()) {
            swerve.driveHeading(
                    forwardTranslation,
                    sideTranslation,
                    swerve.getAlignAngle(),
                    true);
        } else {
            swerve.drive(
                    corrected.vxMetersPerSecond,
                    corrected.vyMetersPerSecond,
                    corrected.omegaRadiansPerSecond,
                    true,
                    false);
        }
    }
}

