package frc.robot.commands.gpm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Outtake.Outtake;


public class IntakeCoral extends Command {
    Intake intake;
    Indexer indexer; 
    Outtake outtake; 
    ArmComp arm; 
    Elevator elevator; 
    public IntakeCoral(Intake intake, Indexer indexer, Elevator elevator, Outtake outtake, ArmComp arm) {
        this.intake = intake;
		this.indexer = indexer;
		this.outtake = outtake;
		this.arm = arm;
		this.elevator = elevator;
		addRequirements(intake, indexer, arm, elevator);  
    }
    public void intitialize() {
        intake.unstow();
        intake.startRollers();
        indexer.run();
        outtake.setMotor(.7); 
    }
    public void execute() {
        // Change from last year, so might not work 
        if (outtake != null && outtake.coralLoaded()) {
            indexer.stop();
            intake.stopRollers(); 
            elevator.setSetpoint(1); // TODO: Replace with actual stow constant 
			arm.setSetpoint(1); // TODO: Replace with actual stow constant   
        }
    } 
    public boolean isFinished() {
        return elevator.getPosition()> 1; // TODO: Replace with actual safe setpoint 

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