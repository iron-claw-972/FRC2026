package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;

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

    public Command getLeftDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
            new PathPlannerAuto("LeftSwipeOne"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("LeftSwipeTwo"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("LeftSwipeTwo"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("LeftSwipeTwo"),
            runSpindexerWithAbort()
        );
    }

    public Command getRightDynamicDoubleLiberalSwipe() {
        return new SequentialCommandGroup(
            new PathPlannerAuto("RightSwipeOne"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("RightSwipeTwo"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("RightSwipeTwo"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("RightSwipeTwo"),
            runSpindexerWithAbort()
        );
    }

    private Command runSpindexerWithAbort() {
        // should race: when a command finnishes (will always be the wait until command) then we will end the command
        // return new RunSpindexer(spindexer, turret, hood, intake)
        // .raceWith(new WaitUntilCommand(() -> spindexer.spinningAir())
        // );

        return new ParallelDeadlineGroup(new WaitUntilCommand(() -> spindexer.spinningAir()), new RunSpindexer(spindexer, turret, hood, intake));
    }
}