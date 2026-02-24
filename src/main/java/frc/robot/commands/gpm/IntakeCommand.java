package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command{
    private Intake intake;

    public IntakeCommand(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extend();
        intake.spinStart();
    }

    @Override
    public void end(boolean interrupted) {
        intake.intermediateExtend();
        intake.spinStop();
    }

}
