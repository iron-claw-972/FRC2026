package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ClimbDriveCommand extends SequentialCommandGroup{

    public ClimbDriveCommand(LinearClimb climb, Drivetrain drive){
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> climb.climbPosition()),
                new DriveToPose(drive, () -> FieldConstants.getClimbLocation())
            )
        );
    }


}
