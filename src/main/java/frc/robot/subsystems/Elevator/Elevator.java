package frc.robot.subsystems.Elevator;

public abstract class Elevator {
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
