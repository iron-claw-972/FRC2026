package frc.robot.commands.gpm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;

public class RunSpindexer extends Command {
    private Spindexer spindexer;
    private Turret turret;

    private Debouncer jam_debouncer = new Debouncer(SpindexerConstants.JAM_DEBOUNCE_TIME, DebounceType.kRising); // if their is jam I would think this is 0 -> 1
    private Debouncer reversing_debouncer = new Debouncer(SpindexerConstants.REVERSE_DEBOUNCE_TIME, DebounceType.kFalling); // if there is a release in time Idk what it would be (kfalling vs krising)

    private boolean reversing = false;
    public RunSpindexer(Spindexer spindexer, Turret turret) {
        this.spindexer = spindexer;
        this.turret = turret;
        addRequirements(spindexer, turret);
    }

    @Override
    public void execute() {
        if (!turret.atSetpoint()) {
            spindexer.stopSpindexer();
            reversing = false;
            return; // this is so the balls don't fly out when unaligned
        }
        boolean jammed = spindexer.getStatorCurrent() > SpindexerConstants.JAM_CURRENT_THRESHOLD;
        if (jam_debouncer.calculate(jammed)) {
            reversing = true;
            System.out.println("Reversing the spindexer for Anti-Jam");
        }
        if (!reversing) {
            spindexer.maxSpindexer();
        } else {
            spindexer.reverseSpindexer();
            if (reversing_debouncer.calculate(reversing)) {
                reversing = false;
                reversing_debouncer.calculate(false);
                jam_debouncer.calculate(false);
            }
        }
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
