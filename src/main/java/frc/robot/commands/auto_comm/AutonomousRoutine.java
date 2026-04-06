package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.spindexer.Spindexer;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutonomousRoutine extends Command {
    private final String name;
    private final Spindexer spindexer;
    private List<PathPlannerAuto> autos;
    private SequentialCommandGroup commandGroup;
    private boolean autosModified = false;
    private int i = 0;

    public AutonomousRoutine(String name, Spindexer spindexer) {
        this.name = name;
        this.spindexer = spindexer;
        this.autos = initializeAutos(name);
        initializeCommandGroup();
    }

    public String getName() {
        return name;
    }

    private List<PathPlannerAuto> initializeAutos(String name) {
        List<PathPlannerAuto> autos = new ArrayList<>();
        try {
            switch (name) {
                case "Routine A":
                    autos.add(new PathPlannerAuto("Speaker Left 2 Note"));
                    autos.add(new PathPlannerAuto("Four Note Auto"));
                    break;
                case "Routine B":
                    // autos.add(new PathPlannerAuto("(pivot) Front-Speaker to Note"));
                    break;
                default:
                    autos.add(new PathPlannerAuto("BASIC"));
                    break;
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing autos: " + e.getMessage());
        }
        return autos;
    }

    private void initializeCommandGroup() {
        try {
            List<Command> commands = new ArrayList<>();
            for (PathPlannerAuto auto : autos) {
                commands.add(new PathPlannerAuto(auto.getName()));  // Ensure new instances
            }
            commandGroup = new SequentialCommandGroup(commands.toArray(new Command[0]));
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing commandGroup: " + e.getMessage());
        }
    }

    @Override
    public void initialize() {
        // Ensure initial command group scheduling
        scheduleCommandGroup();
    }

    @Override
    public void execute() {
        // Debug statements to track vision state and autos modification
        System.out.println("Spinning Air: " + spindexer.spinningAir());
        System.out.println("Autos Modified: " + autosModified);
        i+=1;
        // Check if we are trying to shoot nothing and autos have not been modified
        if (!spindexer.spinningAir() && !autosModified && i == 300) {
            modifyAutosBasedOnSpindexer();
            autosModified = true;
            cancelAndScheduleCommandGroup();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (commandGroup != null) {
            commandGroup.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        // Ensure the command group is not finished prematurely
        return commandGroup != null && commandGroup.isFinished();
    }
    private void modifyAutosBasedOnSpindexer() {
        try {
            // Create a new list to store the modified autos
            List<PathPlannerAuto> modifiedAutos = new ArrayList<>(autos);

            // Example modification: Replace the current path
            PathPlannerAuto autoToMove = new PathPlannerAuto("Speaker Front 3 Note");
            modifiedAutos.removeIf(auto -> auto.getName().equals("Four Note Auto"));
            modifiedAutos.add(autoToMove);

            // Update the autos list with modified paths
            autos = new ArrayList<>(modifiedAutos);

            // Reinitialize the command group with the modified autos
            initializeCommandGroup();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error modifying autos based on spindexer: " + e.getMessage());
        }
    }

    private void scheduleCommandGroup() {
        if (commandGroup != null) {
            CommandScheduler.getInstance().schedule(commandGroup);
        }
    }

    private void cancelAndScheduleCommandGroup() {
        if (commandGroup != null) {
            CommandScheduler.getInstance().cancel(commandGroup); // Cancel the current command group
        }
        scheduleCommandGroup();
    }
}