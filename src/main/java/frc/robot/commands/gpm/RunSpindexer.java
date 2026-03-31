package frc.robot.commands.gpm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;

public class RunSpindexer extends Command {
    private Spindexer spindexer;
    private Turret turret;
    private Hood hood;

    private Debouncer jam_debouncer = new Debouncer(SpindexerConstants.JAM_DEBOUNCE_TIME, DebounceType.kRising); // if there is jam I would think this is 0 -> 1

    private boolean reversing = false;
    private Timer reverseTimer = new Timer();
    private boolean wasHoodForcedDown = false;

    public RunSpindexer(Spindexer spindexer, Turret turret, Hood hood) {
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;

        addRequirements(spindexer);
    }

        // public RunSpindexer(Spindexer spindexer) {
        // this.spindexer = spindexer;
        // addRequirements(spindexer);
    // }

    @Override
    public void execute() {
        boolean hoodForcedDown = hood.getHoodForcedDown();
        
        if (wasHoodForcedDown && !hoodForcedDown) {
            spindexer.maxSpindexer();
        }
        wasHoodForcedDown = hoodForcedDown;
        
        if (!turret.atSetpoint() || hoodForcedDown) {
            spindexer.stopSpindexer();
            reversing = false;
            return; // this is so the balls don't fly out when unaligned
        }
        boolean jammed = spindexer.getStatorCurrent() > SpindexerConstants.JAM_CURRENT_THRESHOLD;
        if (jam_debouncer.calculate(jammed)) {
            reversing = true;
            reverseTimer.reset();
            reverseTimer.start();
        }
        if (!reversing) {
            spindexer.maxSpindexer();
        } else {
            spindexer.reverseSpindexer();
            if (reverseTimer.hasElapsed(SpindexerConstants.REVERSE_DEBOUNCE_TIME)) {
                reversing = false;
            }
        }
        SmartDashboard.putBoolean("Spindexer Jamming", reversing);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
        reversing = false;
    }

    // @Override
    // public boolean isFinished() {
    //     return false;  // never ends on its own
    // }
}
