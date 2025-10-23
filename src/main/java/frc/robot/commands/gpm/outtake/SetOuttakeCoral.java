package frc.robot.commands.gpm.outtake;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants.ReefLevel;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;

public class SetOuttakeCoral extends ParallelCommandGroup {
    private Elevator elevator;
    private ArmComp arm;
    private double elevatorSetpoint;
    private double armSetpoint;

    public SetOuttakeCoral(Elevator elevator, ArmComp arm, ReefLevel reefLevel, byte selectedDirection){
        this.elevator = elevator;
        this.arm = arm;
        elevatorSetpoint = reefLevel.elevatorSetpoint;
        armSetpoint = selectedDirection == -1 ? ArmConstants.L4_SETPOINT_LEFT : reefLevel.armSetpoint;
        addCommands(
            new SetElevator(elevator, elevatorSetpoint),
            new SetArm(arm, armSetpoint) 
        );
    }
    
}
