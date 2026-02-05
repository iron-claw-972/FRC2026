package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    final int rightID = 1;
    final int leftID = 2;
    final int rollerID = 3;

    private TalonFX rollerMotor; 
    private TalonFX rightMotor; //leader
    private TalonFX leftMotor; //invert this one
    private double maxExtension; // this should go in a constants file
    private double startingPoint; // this should go in a constants file
    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);


    public Intake() {
       // set actual IDs
        rightMotor = new TalonFX(rightID);
        leftMotor = new TalonFX(leftID);
        rollerMotor = new TalonFX(rollerID); 

        // right motor configs
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        var slot0RightConfigs = rightConfig.Slot0;
        //find values later
        //friction, maybe?
        slot0RightConfigs.kP = 0;
        slot0RightConfigs.kI = 0;
        slot0RightConfigs.kD = 0;
        slot0RightConfigs.kV = 0;
        slot0RightConfigs.kA = 0;

        // left motor configs
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        var slot0LeftConfigs = leftConfig.Slot0;

        slot0LeftConfigs.kP = 0;
        slot0LeftConfigs.kI = 0;
        slot0LeftConfigs.kD = 0;
        slot0LeftConfigs.kV = 0;
        slot0LeftConfigs.kA = 0;


        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(rightConfig);

        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        leftMotor.getConfigurator().apply(leftConfig);


        //Follower follower = new Follower(rightMotor.getDeviceID(), true);
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        

        SmartDashboard.putData("Extend Intake", new InstantCommand(() -> extend()));
        SmartDashboard.putData("Retract Intake", new InstantCommand(() -> retract()));

       
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Position:", getPosition());
    }

    public void simulationPeriodic(){

    }

    /**
     * in rotations
     * @param setpoint
     */
    public void setPosition(double setpoint) {
        rightMotor.setControl(voltageRequest.withPosition(setpoint));
    }

    /**
     * gets the position in rotations
     */
    public double getPosition(){
        double position = rightMotor.getPosition().getValueAsDouble();
        return position;
    }

    public void spin(double speed) {
        rollerMotor.set(0.2);
    }

    public void extend() {
       setPosition(maxExtension);

    }
    public void retract(){
        setPosition(startingPoint);
        
    }
    
    
}
