package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeBase extends SubsystemBase {
    public abstract void setSetpoint(double setPoint);
    public abstract void setFlyWheel(double speed);
    public abstract double getPosition();
    public abstract double getBaseMotorVelocity();
    public abstract double getFlyWheelVelocity();
    public abstract boolean atSetpoint();
    public abstract boolean flyWheelSpinning();
}
