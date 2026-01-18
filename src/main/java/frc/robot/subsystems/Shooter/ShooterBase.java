package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ShooterBase extends SubsystemBase{
    
    public abstract void setFeeder(double power);
    public abstract void setShooter(double power);

    public void stopFeeder(){
        setFeeder(0);
    };
    public void stopShooter(){
        setShooter(0);
    };

    public abstract double getFeederVelocity();
    public abstract double getShooterVelcoity();

}
