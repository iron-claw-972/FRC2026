package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmBase extends SubsystemBase {
    
    public abstract void setSetpoint(double setpoint);

    public abstract double getAngle();

    public abstract boolean atSetpoint();

    // periodic()?

    public double getAppliedVoltage(){
         
    }

    public void resetAbsolute(){

    }

    public boolean canMoveElevator(){

    }


}
