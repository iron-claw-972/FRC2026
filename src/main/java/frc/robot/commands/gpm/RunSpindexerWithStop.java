package frc.robot.commands.gpm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;

public class RunSpindexerWithStop extends Command {
    private Spindexer spindexer;
    private Turret turret;
    private Hood hood;
    private Intake intake;

    private Debouncer jam_debouncer = new Debouncer(SpindexerConstants.JAM_DEBOUNCE_TIME, DebounceType.kRising); // if there is jam I would think this is 0 -> 1

    private boolean reversing = false;
    private boolean wasHoodForcedDown = false;

    private Timer reverseTimer = new Timer();

    private double storedIntakeSpeed = 0.0;


    private Timer runTimer = new Timer();

    private Debouncer debouncer = new Debouncer(0.3, DebounceType.kRising);
    
    public RunSpindexerWithStop(Spindexer spindexer, Turret turret, Hood hood, Intake intake) {
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        wasHoodForcedDown = hood.getHoodForcedDown();
        runTimer.reset();
        runTimer.start();
    }

    @Override
    public void execute() {
        boolean hoodForcedDown = hood.getHoodForcedDown();
        
        if (wasHoodForcedDown && !hoodForcedDown) {
            spindexer.maxSpindexer();
        }
        wasHoodForcedDown = hoodForcedDown;
        
        if (!turret.atSetpoint() || hoodForcedDown || spindexer.noIndexing) {
            spindexer.stopSpindexer();
            reversing = false;
            return; // this is so the balls don't fly out when unaligned
        }
        boolean jammed = spindexer.getSubsystemStatorCurrent() / 2 > SpindexerConstants.JAM_CURRENT_THRESHOLD;
        Logger.recordOutput("SpindexerJammed", jammed);
        if (jam_debouncer.calculate(jammed)) {
            Logger.recordOutput("SpindexerJammedDebounced", jammed);

            reversing = true;
            reverseTimer.reset();
            reverseTimer.start();
            storedIntakeSpeed = intake.getSpeed();
        }
        if (!reversing) {
            spindexer.maxSpindexer();
        } else {
            spindexer.reverseSpindexer();

            if (intake.getPosition() > IntakeConstants.INTERMEDIATE_EXTENSION + 1.0) {
                intake.spinReverse();
            } else {
                intake.extend();
            }

            if (reverseTimer.hasElapsed(SpindexerConstants.REVERSE_DEBOUNCE_TIME)) {
                reversing = false;
                intake.spin(storedIntakeSpeed);
            }
        }

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Spindexer Jamming", reversing);
        }
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
        reversing = false;
    }

    @Override
    public boolean isFinished() {
        return runTimer.hasElapsed(1.0) && debouncer.calculate(spindexer.spinningAir());
    }
}
