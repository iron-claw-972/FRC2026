package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class SetElevator extends Command {
    private Elevator elevator; 
    private double setpoint; 
    public SetElevator(Elevator elevator, double setpoint) {
        this.elevator = elevator; 
        this.setpoint = setpoint; 
        addRequirements(elevator);
    }

    public void initialize() {
        if (elevator != null) {
            elevator.setSetpoint(setpoint); 
        }
    }
    public boolean isFinished() {
        return elevator == null || elevator.atSetpoint(); 
    }
}

