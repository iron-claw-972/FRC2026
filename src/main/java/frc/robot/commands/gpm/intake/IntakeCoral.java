package frc.robot.commands.gpm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Outtake.Outtake;

/**
 * Intakes coral from ground
 */
public class IntakeCoral extends Command {
    private Intake intake;
    private Indexer indexer; 
    private Outtake outtake; 
    private ArmComp arm; 
    private Elevator elevator; 

    /**
     * Creates a command to intake the coral from ground
     * 
     * @param intake Intake subsystem
     * @param indexer Indexer subsystem
     * @param elevator Elevator subsystem
     * @param outtake Outtide subsystem
     * @param arm Arm subsystem
     */
    public IntakeCoral(Intake intake, Indexer indexer, Elevator elevator, Outtake outtake, ArmComp arm) {
        this.intake = intake;
		this.indexer = indexer;
		this.outtake = outtake;
		this.arm = arm;
		this.elevator = elevator;
		addRequirements(intake, indexer, arm, elevator);  
    }
    public void initialize(){
		elevator.setSetpoint(ElevatorConstants.INTAKE_SETPOINT); 
		arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
        intake.unstow();
        intake.startRollers();
        indexer.run();
        outtake.setMotor(0.7); 
    }

    public void execute(){
        if (outtake != null && outtake.coralLoaded()) {
            indexer.stop(); 
            intake.stopRollers(); 
            elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT);   
        }
    }

    public boolean isFinished(){
        return elevator.getPosition() > ElevatorConstants.SAFE_SETPOINT;
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            arm.setSetpoint(ArmConstants.STOW_SETPOINT);
            outtake.setMotor(0.02); 
        } else {
            outtake.setMotor(0.0); 
        }
        intake.stopRollers();
		intake.stow();
		indexer.stop();
    }
}