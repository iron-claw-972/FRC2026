package frc.robot.commands.gpm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;

public class IntakeAlgae extends Command {
    private static final double ARM_ALGAE_SETPOINT = 0;
    
    
    private static final double ELEVATOR_ALGAE_HIGH_SETPOINT = 0;
    private static final double ELEVATOR_ALGAE_LOW_SETPOINT = 0;


    private Outtake outtake;
    private ArmComp arm;
    private Elevator elevator;

    private boolean level; //1 is high, 0 is low


    public IntakeAlgae(Outtake outtake, ArmComp arm, Elevator elevator, boolean level){
        this.outtake = outtake;
        this.arm = arm;
        this.level = level;
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        arm.setSetpoint(ARM_ALGAE_SETPOINT);
        if (level){
            elevator.setSetpoint(ELEVATOR_ALGAE_HIGH_SETPOINT);
        } else {
            elevator.setSetpoint(ELEVATOR_ALGAE_LOW_SETPOINT);
        }
    }

    @Override
    public void execute(){
        if (elevator.atSetpoint() && arm.atSetpoint()){
            outtake.reverse();
        }
    }

    public void end(boolean interrupted){
        // elevator.setSetpoint(0.0);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        // elevator.setArmStowed(); Maybe? will this break?
        outtake.setMotor(-0.01); //to keep algae in
        //TODO maybe add lower elevator setpoints to lower center of mass height
    }
}