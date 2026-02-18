package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DetectedObject;

public class AcquireGamePiece extends SequentialCommandGroup {
    /**
     * Intakes a game piece
     *
     * @param gamePiece The supplier for the game piece to intake
     * @param drive     The drivetrain
     */
    public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive) {
        // TODO: Replace DoNothing with next year's intake command
        addCommands(new DoNothing().deadlineFor(new DriveToGamePiece(gamePiece, drive)));
    }
}