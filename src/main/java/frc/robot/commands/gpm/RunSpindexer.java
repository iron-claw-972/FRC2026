package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class RunSpindexer extends Command {
    private Spindexer spindexer;
    private Turret turret;

    public RunSpindexer(Spindexer spindexer, Turret turret){
        this.spindexer = spindexer;
        this.turret = turret;
        
        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        //if (turret.atSetpoint()){
            spindexer.maxSpindexer();
        // } else{
        //     spindexer.stopSpindexer();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
    }
}
