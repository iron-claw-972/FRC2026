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
    private Debouncer jam_debouncer = new Debouncer(0.3, DebounceType.kFalling);
    private boolean reversing = false;
    public RunSpindexer(Spindexer spindexer, Turret turret){
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
        if (jammed) {
            reversing = true;
        }

        if (reversing) {
            spindexer.reverseSpindexer();
            
            if (jam_debouncer.calculate(!jammed)) {
                reversing = false;
            }
        } else {
            spindexer.maxSpindexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
        reversing = false;
    }

    @Override
    public boolean isFinished() {
        return false;  // never ends on its own
    }
}
