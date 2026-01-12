package frc.robot.commands.gpm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
                new Translation3d(Units.inchesToMeters(156.8), 4.035, Units.inchesToMeters(72)),
                4);

        drive.setIsAlign(true);
        hood.setSetpoint(target_state.pitch());
        shooter.setShooter(target_state.speed());
        drive.setAlignAngle(target_state.yaw().getRadians());
    }

    @Override
    public void execute() {
        target_state = ShooterPhysics.getShotParams(
                new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
                        drive.getChassisSpeeds().vyMetersPerSecond),
                drive.getPose().getTranslation(),
                new Translation3d(Units.inchesToMeters(156.8), 4.035, Units.inchesToMeters(72)),
                4);

        hood.setSetpoint(target_state.pitch());
        shooter.setShooter(target_state.speed());
        drive.setAlignAngle(target_state.yaw().getRadians());


        if (hood.atSetpoint()) {
            shooter.setFeeder(1);
        }
    }

    @Override
    public void end(boolean canceled) {
        drive.setIsAlign(false);
        shooter.setFeeder(0);
    }
}
