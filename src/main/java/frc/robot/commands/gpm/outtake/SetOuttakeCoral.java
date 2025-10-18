package frc.robot.commands.gpm.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;

public class SetOuttakeCoral extends ParallelCommandGroup {
    private Elevator elevator;
    private ArmComp arm;
    private String reefLevel;

    public SetOuttakeCoral(Elevator elevator, ArmComp arm, String reefLevel){
        addCommands(
        
        );
    }
    
}
