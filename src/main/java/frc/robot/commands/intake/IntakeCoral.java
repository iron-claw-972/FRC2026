package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoral extends Command{
    Intake intake;
    //TODO add when other subsystems are made
    // Indexer indexer; 
    // Elevator elevator;

    public IntakeCoral(Intake intake) // Indexer indexer, Elevator elevator) {
        this.intake = intake;
        //TODO add when other subsystems are made
        // this.elevator = elevator;
        // this.indexer = indexer;
        addRequirements(intake); //, indexer, elevator);
        
    }

    @Override
    public final void initialize() {
        intake.unstow();
        intake.startRollers();
        
    }

    @Override
    public final void execute() {
    }

    @Override
    public final void end(boolean interrupted) {
    }

    @Override
    public final boolean isFinished() {
        return true;
    }
}
