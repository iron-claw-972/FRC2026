package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.gpm.IntakeMovementCommand;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class DynamicAutoBuilder {

    private final Spindexer spindexer;
    private final Turret turret;
    private final Hood hood;
    private final Intake intake;
    private boolean seizing = false;

    public DynamicAutoBuilder(Spindexer spindexer, Turret turret, Hood hood, Intake intake) {
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
    }

    public Command getLeftDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
                autoStart(),

                new PathPlannerAuto("LeftSwipeOne"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("LeftSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("LeftSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("LeftSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot());

    }

    public Command getRightDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
                autoStart(),

                new PathPlannerAuto("RightSwipeOne"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot(),

                new PathPlannerAuto("RightSwipeTwo"),
                runSpindexerWithAbort(),
                afterStopShoot());
    }

    public Command afterStopShoot() {
        return new SequentialCommandGroup(
                // stop intake seizure
                new InstantCommand(() -> {
                    seizing = false;
                }),
                // extend intake
                new InstantCommand(() -> intake.extend()),
                // reset spindexer
                new InstantCommand(() -> spindexer.resetSpindexer()),
                // hood down
                new InstantCommand(() -> hood.forceHoodDown(true))

        );
    }

    public Command autoStart() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        // hood down
                        new InstantCommand(() -> hood.forceHoodDown(true)),
                        // extend intake
                        new InstantCommand(() -> intake.extend())),
                // spin intake rollers
                new InstantCommand(() -> intake.spinStart()));
    }

    private Command runSpindexerWithAbort() {
        // should race: when a command finnishes (will always be the wait until command)
        // then we will end the command
        // return new RunSpindexer(spindexer, turret, hood, intake)
        // .raceWith(new WaitUntilCommand(() -> spindexer.spinningAir())
        // );

        // return new ParallelDeadlineGroup(new WaitUntilCommand(() ->
        // spindexer.spinningAir()), new RunSpindexer(spindexer, turret, hood, intake));
        return new ParallelCommandGroup(new ParallelDeadlineGroup(
                new WaitCommand(5.0),
                new ParallelCommandGroup(
                        // start spindexer
                        new RunSpindexer(spindexer, turret, hood, intake),
                        new InstantCommand(() -> {
                            seizing = true;
                            CommandScheduler.getInstance()
                                    .schedule(new IntakeMovementCommand(intake).until(() -> !seizing));
                        }),
                        // hood up
                        new InstantCommand(() -> hood.forceHoodDown(false)))

        ));
    }

}