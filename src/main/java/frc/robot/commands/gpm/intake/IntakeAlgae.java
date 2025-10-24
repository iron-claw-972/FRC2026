package frc.robot.commands.gpm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;

public class IntakeAlgae extends Command {
    
    private Outtake outtake;
    private ArmComp arm;
    private Elevator elevator;

    // If true, intaking from top, if false, intaking from bottom 
    private boolean isTop; 

    public IntakeAlgae(Outtake outtake, ArmComp arm, Elevator elevator, boolean isTop){
        this.outtake = outtake;
        this.arm = arm;
        this.elevator = elevator;
        this.isTop = isTop; 
    }

    @Override 
    public void initialize(){
        arm.setSetpoint(ArmConstants.ALGAE_NET_SETPOINT_1);
        elevator.setSetpoint(isTop ? ElevatorConstants.TOP_ALGAE_SETPOINT : ElevatorConstants.BOTTOM_ALGAE_SETPOINT); 
    }

    @Override
    public void execute(){
        if (elevator.atSetpoint() && arm.atSetpoint()){
            outtake.reverse();
        }
    }

    public void end(boolean interrupted){
        // To ensure algae remains intaked
        outtake.setMotor(-0.01); 
    }
}