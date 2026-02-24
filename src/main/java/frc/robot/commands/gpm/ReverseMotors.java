package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import lib.controllers.PS5Controller.PS5Button;

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
        intake.spinReverse();
        spindexer.reverseSpindexer();
    }

    @Override
    public void end(boolean interrupted){
        intake.spinStart();
        spindexer.maxSpindexer();
    }

}
