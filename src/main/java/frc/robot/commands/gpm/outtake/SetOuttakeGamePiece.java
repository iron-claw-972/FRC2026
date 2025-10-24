package frc.robot.commands.gpm.outtake;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants.OuttakeLocation;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;

public class SetOuttakeGamePiece extends ParallelCommandGroup {
    private Elevator elevator;
    private ArmComp arm;
    private double elevatorSetpoint;
    private double armSetpoint;

    public SetOuttakeGamePiece(Elevator elevator, ArmComp arm, OuttakeLocation outtakeLocation) {
        this(elevator, arm, outtakeLocation, 0);
    }

    public SetOuttakeGamePiece(Elevator elevator, ArmComp arm, OuttakeLocation outtakeLocation, int selectedDirection){
        this.elevator = elevator;
        this.arm = arm;
        elevatorSetpoint = outtakeLocation.elevatorSetpoint;
        armSetpoint = selectedDirection == -1 ? ArmConstants.L4_SETPOINT_LEFT : outtakeLocation.armSetpoint;
        addCommands(
            new SetElevator(elevator, elevatorSetpoint),
            new SetArm(arm, armSetpoint) 
        );
    }
    
}
