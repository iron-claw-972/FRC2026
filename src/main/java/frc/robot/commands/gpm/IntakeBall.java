package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class IntakeBall extends Command {
    private Intake intake;
    private Shooter shooter;

    private enum Phase{
        Intaking, Acquiring, Acquired, Done
    }

    private final Timer acquiredTimer = new Timer();

    private Phase phase;

    public IntakeBall(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize(){
        intake.setFlyWheel();
        shooter.setFeeder(0.2);
        intake.setSetpoint(IntakeConstants.INTAKE_ANGLE);
        phase = Phase.Intaking;
    }

    @Override
    public void execute(){
        if (shooter.ballDetected()) {
            acquiredTimer.start();
            phase = Phase.Acquiring;
        }
        switch (phase){
            case Intaking:
            break;
            case Acquiring:
                if (acquiredTimer.get() > 1.0){
                    phase = Phase.Acquired;
                }
                shooter.setFeeder(0.05);
            break;
            case Acquired:
                intake.stopFlyWheel();
                shooter.setFeeder(0);
                intake.setSetpoint(IntakeConstants.STOW_ANGLE);
                phase = Phase.Done;
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
        shooter.setFeeder(0);
    }
}
