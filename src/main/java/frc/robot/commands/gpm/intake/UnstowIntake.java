package frc.robot.commands.gpm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class UnstowIntake extends Command {
    private Intake intake;
    Indexer indexer; 
    Outtake outtake; 
    ArmComp arm; 
    Elevator elevator;

    public UnstowIntake(Intake intake, Indexer indexer, Elevator elevator, ArmComp arm, Outtake outtake){
        this.intake = intake; 
        this.indexer = indexer;
		this.outtake = outtake;
		this.arm = arm;
		this.elevator = elevator;
        addRequirements(intake, indexer, arm, elevator);
    }

    public void initialize(){
        intake.unstow();
        intake.startRollers();
        indexer.run();
        outtake.setMotor(.7); 
    }

    public void execute(){
        if (outtake != null && outtake.coralLoaded()) {
            indexer.stop();
            intake.stopRollers(); 
            elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT); 
			arm.setSetpoint(ArmConstants.STOW_SETPOINT);  
        }
    }

    public boolean isFinished(){
        return false; 
    }

    public void end(boolean interrupted) {
        intake.stopRollers();
		intake.stow();
		indexer.stop();
		if(outtake != null){
			outtake.setMotor(0.02);
		}
    }

}
