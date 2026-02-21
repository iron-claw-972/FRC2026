package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import lib.controllers.PS5Controller.PS5Button;

public class ReverseMotors extends Command {
    // Reverse motors (intake, spindexer, shooter: shoot out constant speed)

        // if (intake != null && spindexer != null && shooter != null){
        //     driver.get(PS5Button.CIRCLE).onTrue(new InstantCommand(() ->{
        //         intake.spinReverse();
        //         // reverse spindexer
        //         shooter.
        //     }))
        // }
    private Intake intake;
    private Spindexer spindexer;
    private Shooter shooter;


    public ReverseMotors(Intake intake, Spindexer spindexer, Shooter shooter){
        this.intake = intake;
        this.spindexer = spindexer;

        addRequirements(intake, spindexer, shooter);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        intake.spinReverse();
        spindexer.reverseSpindexer();
        shooter.setShooter(ShooterConstants.SHOOTER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted){
        intake.spinStop();
        spindexer.stopSpindexer();
        shooter.setShooter(0);
    }

}
