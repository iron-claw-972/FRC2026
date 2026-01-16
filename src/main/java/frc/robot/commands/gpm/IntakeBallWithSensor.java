package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Shooter.ShooterReal;
import frc.robot.subsystems.intake.IntakeReal;

public class IntakeBallWithSensor extends Command {
    private IntakeReal intake;
    private ShooterReal shooter;

    private enum Phase {
        Intaking, Loaded
    }

    private Phase phase;

    public IntakeBallWithSensor(IntakeReal intake, ShooterReal shooter){
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.setFlyWheel();
        intake.setSetpoint(IntakeConstants.INTAKE_ANGLE);
        shooter.setFeeder(0.3);

        phase = Phase.Intaking;
    }

    @Override
    public void execute() {
        if(shooter.ballDetected) {
            phase = Phase.Loaded;
        }
        switch (phase) {
            case Intaking:
            if(shooter.ballDetected){
                shooter.setFeeder(0.0);

                intake.stopFlyWheel();
                intake.setSetpoint(IntakeConstants.STOW_ANGLE);
                phase = Phase.Loaded;
            }
            case Loaded:
            break;
        }
    }

    @Override
    public boolean isFinished(){
        return phase == Phase.Loaded && intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        //in case its interrupted
        shooter.setFeeder(0.0);
        intake.setSetpoint(IntakeConstants.STOW_ANGLE);
        intake.stopFlyWheel();
    }
}
