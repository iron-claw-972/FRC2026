package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class OperatorControllerConfig {
    private final BooleanSupplier slowModeSupplier = () -> false;
    private boolean intakeBoolean = true;
    private Command autoShoot = null;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;
    private LinearClimb climb;

    public OperatorControllerConfig(
        Drivetrain drive,
        Shooter shooter,
        Turret turret,
        Hood hood,
        Intake intake,
        Spindexer spindexer,
        LinearClimb climb
    ) {
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.spindexer = spindexer;
        this.climb = climb;
    }
    
}