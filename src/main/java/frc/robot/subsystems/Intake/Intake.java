package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Intake extends SubsystemBase {
    private TalonFX rollerMotor; 
    private TalonFX rightMotor;
    private TalonFX leftMotor; //invert this one
    private double position;
    private double maxExtension;
    private double current;

    public Intake() {
       // set actual IDs
        rollerMotor = new TalonFX(0);
        rightMotor = new TalonFX(0);
        leftMotor = new TalonFX(0);

    }
    public void periodic() {
        // current threshold + PID stuff
    }
    public void simulationPeriodic(){

    }
    public void setPosition(double position) {


    }
    public void spin(double speed) {
        rollerMotor.set(0.2);
    }


    public void extend() {
       setPosition(maxExtension);

    }
    public void retract(){
        setPosition(0);
        
    }
    
}
