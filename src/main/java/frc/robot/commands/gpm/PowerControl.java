package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.PowerControl.Battery;
import frc.robot.subsystems.PowerControl.EMABreaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class PowerControl extends Command {
    // my beautiful power control subsystems
    private EMABreaker breaker;
    private Battery battery;
    // the real subsystems
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;

    public SeverityLevel severityLevel;

    public enum SeverityLevel {
        SEVERITY_LVL_ZERO,
        SEVERITY_LVL_ONE,
        SEVERITY_LVL_TWO,
        SEVERITY_LVL_THREE,
        SEVERITY_LVL_FOUR,
        SEVERITY_LVL_FIVE,
    }

    public PowerControl(
        EMABreaker breaker, // pc
        Battery battery, // pc
        Drivetrain drivetrain, // main draw
        Shooter shooter, // aiming (vital)
        Turret turret, // aiming
        Hood hood, // aiming
        Intake intake, // bps
        Spindexer spindexer // bps
    ) {
        this.breaker = breaker;
        this.battery = battery;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.spindexer = spindexer;

        addRequirements(breaker, battery); // not sure if I'll need requirement access for setting new current limits
    }

    @Override
    public void initialize() {
        severityLevel = SeverityLevel.SEVERITY_LVL_ZERO;
    }

    @Override
    public void execute() {
        double[] worstFilter = breaker.percentageMaxUsage();
        double percentage = worstFilter[0]; // percentage of current average until we trip breaker
        double tau = worstFilter[1]; // how quickly this issue is happenning and if we need to respond quickly

        // Some logic here
    }

    @Override
    public void end(boolean interupted) {
        severityLevel = SeverityLevel.SEVERITY_LVL_ZERO; // in the case of disabling this command we shoud reset its effects
    }
}

