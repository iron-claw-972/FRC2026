package frc.robot.commands.gpm.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;

public class OuttakeAlgae extends Command {

    private static final double ELEVATOR_ALGAE_OUTTAKE_SETPOINT = 0;
    private static final double ARM_ALGAE_OUTTAKE_SETPOINT = 0;

    private Outtake outtake;
    private ArmComp arm;
    private Elevator elevator;

    public OuttakeAlgae(Outtake outtake, ArmComp arm, Elevator elevator){
        this.outtake = outtake;
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        arm.setSetpoint(ARM_ALGAE_OUTTAKE_SETPOINT);
        elevator.setSetpoint(ELEVATOR_ALGAE_OUTTAKE_SETPOINT);
    }

    @Override
    public void execute(){
        if (elevator.atSetpoint() && arm.atSetpoint()){
            outtake.outtake();
        }
    }

    public void end(boolean interrupted){
        elevator.setSetpoint(0.0);
        elevator.setArmStowed(); 
        outtake.setMotor(0); //to keep algae in
        //TODO maybe add lower elevator setpoints to lower center of mass height
    }
}