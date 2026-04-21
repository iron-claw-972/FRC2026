package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.gpm.IntakeCommand;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.commands.gpm.RunSpindexerWithStop;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.PowerControl.BreakerConstants;
import frc.robot.subsystems.PowerControl.EMABreaker;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class DynamicAutoBuilder {

    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Intake intake;

    public DynamicAutoBuilder(Spindexer spindexer, Turret turret, Hood hood, Intake intake) {
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
    }

    /*
     * Autos have no named commands within them. They must be added here
     * Still need to make one method to call that four command block in each sequential
     */

    public Command getDynamicDoubleLiberalSwipe(boolean left) {
        return new SequentialCommandGroup(
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeOne" : "RightSwipeOne"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),
                
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort()
        ); 
    }
        
    public Command getDynamicDoubleConservativeSwipe(boolean left) {
        return new SequentialCommandGroup(
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeConservative" : "RightSwipeConservative"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),
                
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort()
        ); 
    }

    public Command getDynamicDoubleShallowSwipe(boolean left) {
        return new SequentialCommandGroup(
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeThree" : "RightSwipeThree"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),

                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort(),
                
                departCommand(),
                new PathPlannerAuto(left ? "LeftSwipeTwo" : "RightSwipeTwo"),
                startShootingCommand(),
                runSpindexerWithAbort()
        ); 
    }

    private Command departCommand() {
        return new InstantCommand(() -> {
            hood.forceHoodDown(true);
            intake.extend();
            intake.spinStart();
            spindexer.stopSpindexer();
        });
    }

    private Command runSpindexerWithAbort() {
        // should race: when a command finnishes (will always be the wait until command)
        // then we will end the command
        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(new
        // WaitCommand(5.0));

        // var timer = new Timer();
        // timer.start();

        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(
        //         new WaitUntilCommand(() -> spindexer.spinningAir() && timer.hasElapsed(3.0)));
        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(new WaitCommand(3.0));
        // return new RunSpindexer(spindexer, turret, hood, intake).until(() -> spindexer.spinningAir() && timer.hasElapsed(3.0));

        // has an isFinnished so it should work
        return new RunSpindexerWithStop(spindexer, turret, hood, intake);

        // return new ParallelDeadlineGroup(new WaitUntilCommand(() ->
        // spindexer.spinningAir()), new RunSpindexer(spindexer, turret, hood, intake));
        // return new ParallelDeadlineGroup(new WaitCommand(5.0), new
        // RunSpindexer(spindexer, turret, hood, intake));
    }

    private Command startShootingCommand() {
        return new InstantCommand(() -> hood.forceHoodDown(false));
    }
}
