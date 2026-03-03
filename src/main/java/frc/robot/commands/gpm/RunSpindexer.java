package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class RunSpindexer extends Command {
    private Spindexer spindexer;
    public RunSpindexer(Spindexer spindexer){
        this.spindexer = spindexer;
        
        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        spindexer.maxSpindexer();
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
    }
}
