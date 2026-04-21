package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.gpm.IntakeCommand;
import frc.robot.commands.gpm.RunSpindexer;
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
    private final EMABreaker breaker;

    public DynamicAutoBuilder(Spindexer spindexer, Turret turret, Hood hood, Intake intake, EMABreaker breaker) {
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.breaker = breaker;
    }

    public Command getLeftDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    hood.forceHoodDown(true);
                    intake.extend();
                    intake.spinStart();
                }),

                new PathPlannerAuto("LeftSwipeOne"),

                new InstantCommand(() -> hood.forceHoodDown(false)),
                runSpindexerWithAbort(),
                new InstantCommand(() -> hood.forceHoodDown(true)),

                new PathPlannerAuto("LeftSwipeTwo"),
                new InstantCommand(() -> hood.forceHoodDown(false)),
                runSpindexerWithAbort(),
                new InstantCommand(() -> hood.forceHoodDown(true)),

                new PathPlannerAuto("LeftSwipeTwo"),
                new InstantCommand(() -> hood.forceHoodDown(false)),
                runSpindexerWithAbort(),
                new InstantCommand(() -> hood.forceHoodDown(true)),

                new PathPlannerAuto("LeftSwipeTwo"),
                new InstantCommand(() -> hood.forceHoodDown(false)),
                runSpindexerWithAbort(),
                new InstantCommand(() -> hood.forceHoodDown(true))
        );       
    }

    public Command getRightDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> hood.forceHoodDown(true)),

                new PathPlannerAuto("RightSwipeOne"),
                runSpindexerWithAbort(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort());
    }

    private Command runSpindexerWithAbort() {
        // should race: when a command finnishes (will always be the wait until command)
        // then we will end the command
        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(new
        // WaitCommand(5.0));

        var timer = new Timer();
        timer.start();

        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(
        //         new WaitUntilCommand(() -> spindexer.spinningAir() && timer.hasElapsed(3.0)));
        // return new RunSpindexer(spindexer, turret, hood, intake).raceWith(new WaitCommand(3.0));
        return new RunSpindexer(spindexer, turret, hood, intake).until(() -> spindexer.spinningAir() && timer.hasElapsed(3.0));


        // return new ParallelDeadlineGroup(new WaitUntilCommand(() ->
        // spindexer.spinningAir()), new RunSpindexer(spindexer, turret, hood, intake));
        // return new ParallelDeadlineGroup(new WaitCommand(5.0), new
        // RunSpindexer(spindexer, turret, hood, intake));
    }
}
