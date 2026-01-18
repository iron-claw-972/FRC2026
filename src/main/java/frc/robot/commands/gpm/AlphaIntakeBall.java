package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeAlpha;

public class AlphaIntakeBall extends Command {
    private IntakeAlpha intake;

    public AlphaIntakeBall(IntakeAlpha intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        intake.setPower(1);
    }

    @Override
    public void end(boolean interrupted){
        intake.setPower(0);
    }
}
