package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Shooter.shooterReal;
import frc.robot.subsystems.intake.IntakeReal;

public class IntakeBall extends Command {
    private IntakeReal intake;
    private shooterReal shooter;

    private enum Phase{
        Intaking, Acquired, Done
    }

    private Phase phase;

    public IntakeBall(IntakeReal intake, shooterReal shooter){
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize(){
        intake.setFlyWheel();
        intake.setSetpoint(IntakeConstants.INTAKE_ANGLE);
        phase = Phase.Intaking;
    }

    @Override
    public void execute(){
        if(shooter.ballDetected()){
            phase = Phase.Acquired;
        }
        switch (phase){
            case Intaking:
            break;
            case Acquired:
                if(shooter.ballDetected()){
                    intake.stopFlyWheel();
                    intake.setSetpoint(IntakeConstants.STOW_ANGLE);
                    phase = Phase.Done;
                }
            break;
            case Done:
            break;
        }
    }

    @Override
    public boolean isFinished(){
        return phase == Phase.Done && intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        //in case its interrupted
        intake.setSetpoint(IntakeConstants.STOW_ANGLE);
        intake.stopFlyWheel();
    }
}
