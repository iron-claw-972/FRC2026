package frc.robot.util;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Simple check on PathPlanner path
 */
public class PathCheck {

    /**
     * Register placeholder commands for testing.
     * These are normally registered in RobotContainer.registerCommands(),
     * but the test doesn't instantiate RobotContainer.
     */
    @BeforeAll
    public static void registerPlaceholderCommands() {
        NamedCommands.registerCommand("Extend intake", new InstantCommand());
        NamedCommands.registerCommand("Intake", new InstantCommand());
        NamedCommands.registerCommand("Climb", new InstantCommand());
    }

    /**
     * Load the path groups.
     * <p>
     * We have had problems with syntax errors in a path.
     */
    @Test
    public void pathGroupLoaderTest() {
        // load the paths
        //   may throw a ParseException; that error will fail this test
        PathGroupLoader.loadPathGroups();
    }
}
