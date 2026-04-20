package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class F1Mode extends Command {
    private Intake intake;
    private Drivetrain drive;
    private Hood hood;
    private Shooter shooter;
    private Turret turret;

    public F1Mode(Turret turret, Shooter shooter, Hood hood, Intake intake, Drivetrain drive) {
        this.intake = intake;
        this.drive = drive;
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;

        addRequirements(intake, drive, hood, shooter, turret);
    }

    @Override
    public void initialize() {
        drive.applyNewModuleCurrents(
            DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT * 1.75,
            DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT * 1.75, 
            DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT * 1.75, 
            DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT * 1.75
        );
        intake.retract(); // pull intake in
        hood.forceHoodDown(true); // force the hood down (spindexer will stop)
        shooter.setNewCurrentLimit(0, 0);
        turret.setNewCurrentLimit(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.applyNewModuleCurrents(
            DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT * 1.00,
            DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT * 1.00, 
            DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT * 1.00, 
            DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT * 1.00
        );
        hood.forceHoodDown(false);
        shooter.setNewCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT, ShooterConstants.SHOOTER_CURRENT_LIMIT);
        turret.setNewCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT, TurretConstants.SUPPLY_CURRENT_LIMIT);
    }
}
