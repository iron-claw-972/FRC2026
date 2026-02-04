package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Intake extends SubsystemBase {
    private TalonFX rollerMotor; 
    private TalonFX rightMotor; //leader
    private TalonFX leftMotor; //invert this one
    private double position;
    private double maxExtension; //rotations
    private PIDController pid;
    private double gearRatio;

    public Intake() {
       // set actual IDs
        rollerMotor = new TalonFX(0);
        rightMotor = new TalonFX(0);
        leftMotor = new TalonFX(0);
        var config = new TalonFXConfiguration();
        var slot0Configs = config.Slot0;
        //find values later
        //friction, maybe?
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;


        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(config);

        leftMotor.getConfigurator().apply(config);
        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
       

    }

    public void periodic() {
        // current threshold + PID stuff
        double motorPosition = getPosition();
        double currentPosition = Units.rotationsToRadians(motorPosition/gearRatio);
        double power = pid.calculate(currentPosition);

     
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

       // add tolerance
       // double check
       if (position == maxExtension) {
        leftMotor.set(0);
        rightMotor.set(0);
       } 

    }
    public void retract(){
        setPosition(0);
        
    }
    
}
