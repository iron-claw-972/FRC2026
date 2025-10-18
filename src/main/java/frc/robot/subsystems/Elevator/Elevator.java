package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {
    public abstract void setArmStowed();
    public abstract void periodic();
    public abstract void simulationPeriodic();
    public abstract void resetEncoder(double height);
    public abstract void updateInputs();
    public abstract double getPosition();
    public abstract double getVelocity(); 
    public abstract double getVoltage();
    public abstract void setSetpoint(double setpoint); 
    public abstract double getSetpoint();
    public abstract boolean atSetpoint();
}
