package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ClimbCommand extends SequentialCommandGroup{

    public ClimbCommand(LinearClimb climb, Drivetrain drive, PS5Controller controller){
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> climb.goUp()),
                new DriveToPose(drive, () -> FieldConstants.getClimbLocation())
            ),
            new InstantCommand(() -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0))
        );
    }


}
