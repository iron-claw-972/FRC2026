package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeMovementCommand extends Command {
    private final Intake intake;
    private final double interval = 0.6; // Change this to make it faster/slower (seconds)

    public IntakeMovementCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.spinStop();
        if ((int) (Timer.getFPGATimestamp() / interval) % 2 == 0) {
            intake.extend(); 
        } else {
            intake.intermediateExtend();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}