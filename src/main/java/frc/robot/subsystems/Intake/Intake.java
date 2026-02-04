package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
    private PIDController pid;
    private double gearRatio;

    public Intake() {
       // set actual IDs
        rollerMotor = new TalonFX(0);
        rightMotor = new TalonFX(0);
        leftMotor = new TalonFX(0);
        pid = new PIDController(0, 0, 0);

    

    }

    public void periodic() {
        // current threshold + PID stuff
        double motorPosition = getPosition();
        double currentPosition = Units.rotationsToRadians(motorPosition/gearRatio);
        double power = pid.calculate(currentPosition);
       // THIS IS THE WRONG MOTOR
        rollerMotor.set(MathUtil.clamp(power, -1, 1));
    }

    public void simulationPeriodic(){

    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getPosition(){
        // position is in rotations
        double position = rollerMotor.getPosition().getValueAsDouble();
        return position;
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
