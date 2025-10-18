package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract class HoodBase extends SubsystemBase{

    /**
     * sets the setpoint for hood pid angle
     * @param setpoint setpoint for hood in radians
     */
    public abstract void setSetpoint(double setpoint);
    
    /**
     * Gets the position of the hood for robot
     * @return Returns position of hood in radians
     */
    public abstract double getPosition();

    /**
     * Gets the velocity of the hood
     * @return
     */
    public abstract double getVelocity();

    /**
     * Returns if hood is at the setpoint from the PID
     * @return boolean representing if hood is a the setpoint
     */
    public abstract boolean atSetpoint();
}