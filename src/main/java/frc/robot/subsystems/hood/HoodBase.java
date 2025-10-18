package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodBase extends SubsystemBase {
    public abstract void setSetpoint(double setPoint);
    public abstract double getPosition();
    public abstract double getVelocity();
    public abstract boolean atSetpoint();
}