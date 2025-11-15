package frc.robot.commands.gpm.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Constants.OuttakeLocation;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.intake.Intake;

/**
 * Scores coral/algae
 */
public class OuttakeGamePiece extends Command {
    private Elevator elevator;
    private ArmComp arm;
    private Intake intake;
    private Outtake outtake;
    private double speed;
    private Timer timer;

    private enum Phase {
        Outtaking,
        InitialLowering,
        FinalLowering,
        Done
    }

    private Phase phase; 
    /**
     * Creates the command to score coral/algae
     * 
     * @param elevator Elevator subsystem
     * @param arm Arm subsystem
     * @param intake Intake subsystem
     * @param outtake Outtake subsystem
     * @param outtakeLocation Location the gamepiece will be outtaked to 
     */

    public OuttakeGamePiece(Elevator elevator, ArmComp arm, Intake intake, Outtake outtake){
        double height = elevator.getPosition(); 
        if (height > ElevatorConstants.L4_SETPOINT + 0.001) {
            speed = OuttakeLocation.L4.speed; 
        } else if (height > ElevatorConstants.L3_SETPOINT + 0.001) {
            speed = OuttakeLocation.L3.speed; 
        } else if (height > ElevatorConstants.L2_SETPOINT + 0.001) {
            speed = OuttakeLocation.L2.speed;
        } else if (height > ElevatorConstants.L1_SETPOINT + 0.001) {
            speed = OuttakeLocation.L1.speed;
        } else {
            speed = OuttakeLocation.L4.speed; 
        }
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.outtake = outtake;
        timer = new Timer();
    }

    public OuttakeGamePiece(Elevator elevator, ArmComp arm, Intake intake, Outtake outtake, OuttakeLocation outtakeLocation){
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.outtake = outtake;
        timer = new Timer();
        speed = outtakeLocation.speed; 
    }

    @Override
    public void initialize(){
        timer.restart();
        outtake.outtake(); 
        phase = Phase.Outtaking;
    }

    @Override
    public void execute(){
        switch (phase){
            case Outtaking:
                if (timer.hasElapsed(0.5)){
                    outtake.setMotor(0);
                    arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                    elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                    phase = Phase.InitialLowering;
                }
                break;
                
            case InitialLowering:
                if (elevator.getPosition() < ElevatorConstants.L2_SETPOINT) {
                    // if the arm is at fully stowed then continue lowering elevator 
                    if (arm.atSetpoint()) {
                        elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                        phase = Phase.FinalLowering;
                    // if arm is not fully stowed, stop elevator and wait until fully stowed then lower elevator
                    } else {
                        // Stop a little bit under L2 to make 
                        elevator.setSetpoint(ElevatorConstants.L2_SETPOINT - .1); 
                    }
                } 
                break;
            case FinalLowering:
                if (elevator.atSetpoint()) {
                    phase = Phase.Done; 
                }
                break;
            case Done: 
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (phase == Phase.Done) {
            return true; 
        } 
        return false; 
    }



}
