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

    public Command getDynamicAuto() {
        return new SequentialCommandGroup(
            new PathPlannerAuto("SwipeOne"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("SwipeTwo"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("SwipeThree"),
            runSpindexerWithAbort(),

            new PathPlannerAuto("SwipeFour"),
            runSpindexerWithAbort()
        );
    }

    private Command runSpindexerWithAbort() {
        return new RunSpindexer(spindexer, turret, hood, intake)
        .raceWith(new WaitUntilCommand(() -> spindexer.spinningAir())
        );
    }
}