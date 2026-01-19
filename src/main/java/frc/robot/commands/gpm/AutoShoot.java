package frc.robot.commands.gpm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterReal;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.HoodBase;
import frc.robot.subsystems.hood.HoodReal;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysics.TurretState;

public class AutoShoot extends Command {
    Drivetrain drive;
    HoodReal hood;
    ShooterReal shooter;
    TurretState target_state;

    boolean atTarget;
    // apex of parabola in meters
    double peakHeight = 10.0;

    public AutoShoot(Drivetrain drive, HoodReal hood, ShooterReal shooter) {
        this.drive = drive;
        this.hood = hood;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        target_state = ShooterPhysics.getShotParams(
                new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
                        drive.getChassisSpeeds().vyMetersPerSecond),
                drive.getPose().getTranslation(),
                FieldConstants.HUB_TRANSLATION3D,
                peakHeight);
                
        hood.setSetpoint(Units.radiansToDegrees(target_state.pitch()));
        shooter.setShooter(target_state.exitVel());
        drive.setIsAlign(true);
        drive.setAlignAngle(target_state.yaw().getRadians());
    }

    @Override
    public void execute() {
        target_state = ShooterPhysics.getShotParams(
                new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
                        drive.getChassisSpeeds().vyMetersPerSecond),
                drive.getPose().getTranslation(),
                FieldConstants.HUB_TRANSLATION3D,
                peakHeight);

        hood.setSetpoint(target_state.pitch());
        shooter.setShooter(target_state.exitVel());
        drive.setAlignAngle(target_state.yaw().getRadians());


        if (hood.atSetpoint() && drive.atAlignAngle() && shooter.atTargetSpeed()) {
            shooter.setFeeder(1);
            shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER);
        } else {
            shooter.setFeeder(0);
        }

        SmartDashboard.putNumber("Target Hood Angle", target_state.pitch());
        SmartDashboard.putNumber("Target Exit Velocity", target_state.exitVel());
    }

    @Override
    public void end(boolean canceled) {
        drive.setIsAlign(false);
        shooter.setFeeder(0);
    }
}
