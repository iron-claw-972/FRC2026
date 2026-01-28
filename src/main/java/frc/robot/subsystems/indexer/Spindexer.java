package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private TalonFX motor;

    public Spindexer(){
        // TODO: change device id
        motor = new TalonFX(1);

        // Smartdashboard commands
        SmartDashboard.putData("Run", new InstantCommand(()-> run()));
        SmartDashboard.putData("Slow", new InstantCommand(()-> slow()));
        SmartDashboard.putData("Reverse", new InstantCommand(()-> reverse()));
        SmartDashboard.putData("Stop", new InstantCommand(()-> stop()));
        
    } 

    public void run(){
        motor.set(1);
    }

    public void slow(){
        motor.set(0.6);
    }

    public void reverse(){
        motor.set(-1);
    }

    public void stop(){
        motor.set(0);
    }
}
