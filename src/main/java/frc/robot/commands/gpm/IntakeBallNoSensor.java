package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Shooter.shooterReal;
import frc.robot.subsystems.intake.IntakeReal;

public class IntakeBallNoSensor extends Command {
    private IntakeReal intake;
    private shooterReal shooter;
    public IntakeBallNoSensor(IntakeReal intake, shooterReal shooter){
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setSetpoint(IntakeConstants.INTAKE_ANGLE);
        intake.setFlyWheel();
        shooter.setFeeder(0.3);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
