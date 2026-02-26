package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;

public class ReverseMotors extends Command {
    private Intake intake;
    private Spindexer spindexer;


    public ReverseMotors(Intake intake, Spindexer spindexer){
        this.intake = intake;
        this.spindexer = spindexer;

        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        intake.extend();
        intake.spinReverse();
        //spindexer.reverseSpindexer();
    }

    @Override
    public void end(boolean interrupted){
        intake.spinStop();
        //spindexer.maxSpindexer();
    }

}
