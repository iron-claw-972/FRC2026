package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class ReverseMotors extends Command {
    private Intake intake;


    public ReverseMotors(Intake intake){
        this.intake = intake;

        addRequirements(intake);
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
        intake.extend();
        intake.spinStart();
        //spindexer.maxSpindexer();
    }

}
