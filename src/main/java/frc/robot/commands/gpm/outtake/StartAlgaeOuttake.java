package frc.robot.commands.gpm.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;

public class StartAlgaeOuttake extends Command {
    
    private static final double ELEVATOR_ALGAE_OUTTAKE_SETPOINT = 0;
    private static final double ARM_ALGAE_OUTTAKE_SETPOINT = 0;

    private Outtake outtake;
    private ArmComp arm;
    private Elevator elevator;

    public StartAlgaeOuttake(Outtake outtake, ArmComp arm, Elevator elevator){
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
    public boolean isFinished(){
        return (arm.atSetpoint() && elevator.atSetpoint());
    }

    public void end(boolean interrupted){
        outtake.setMotor(-0.01); //to keep algae in
    }
}
