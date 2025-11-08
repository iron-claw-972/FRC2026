package frc.robot.commands.gpm.outtake;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants.OuttakeLocation;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.elevator.Elevator;

/**
 * Prepares the robot for outtake 
 */
public class SetOuttakeGamePiece extends ParallelCommandGroup {
    private Elevator elevator;
    private ArmComp arm;
    private double elevatorSetpoint;
    private double armSetpoint;

    /**
     * Creates the command to prepare the robot to outtake manually
     * 
     * @param elevator Elevator subsystem
     * @param arm Arm subsystem
     * @param outtakeLocation Location the gamepiece will be outtaked to 
     */
    public SetOuttakeGamePiece(Elevator elevator, ArmComp arm, OuttakeLocation outtakeLocation) {
        this(elevator, arm, outtakeLocation, 0);
    }
    /**
     * Creates the command to prepare the robot to outtake for vision 
     * @param elevator Elevator subsystem
     * @param arm Arm subsystem
     * @param outtakeLocation Location the gamepiece will be outtaked to 
     * @param selectedDirection 1 if outtaking right branch, -1 if outtaking left branch
     */
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
