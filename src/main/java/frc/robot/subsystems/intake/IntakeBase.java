package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeBase extends SubsystemBase{
    public abstract void setIntakeSetpoint(double position);

    public abstract double getIntakePosition();

    public abstract double getIntakeVelocity();

    public abstract boolean IntakeAtSetpoint();

    public abstract void setFlywheel(double power);

    public abstract double getFlywheelVelocity();

    public abstract void stopFlywheel();
}
