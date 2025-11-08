package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/**
 * Sets the elevator to the desired setpoint
 */
public class SetElevator extends Command {
    private Elevator elevator; 
    private double setpoint; 

    /**
     * Creates the command to set the elevator 
     * 
     * @param elevator Elevator subsystem
     * @param setpoint The height the elevator will go to (in meters)
     */
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

