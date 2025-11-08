package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeBase extends SubsystemBase implements IntakeIO{
    public abstract void setSetpoint(double setPoint);
    public abstract void setFlyWheel();
    public abstract void stopFlyWheel();
    public abstract double getAngle();
    public abstract double getFlyWheelVelocity();
    public abstract boolean atSetpoint();
    public abstract boolean flyWheelSpinning();
}
