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

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    private Mechanism2d mechanism;
    private MechanismLigament2d mechanismLigament2d;
    // set actual IDs
    final int rightID = 1;
    final int leftID = 2;
    final int rollerID = 3;

    private TalonFX rollerMotor; 
    private TalonFX rightMotor; //leader
    private TalonFX leftMotor; //invert this one
    private double maxExtension; // this should go in a constants file
    private double startingPoint; // this should go in a constants file
    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    final MechanismLigament2d extensionLigament;
    final double kMaxRotations = 37.5;
    final double kMaxVisualLength = 0.75;


    public Intake() {
        rightMotor = new TalonFX(rightID);
        leftMotor = new TalonFX(leftID);
        rollerMotor = new TalonFX(rollerID); 

        

        // right motor configs
        TalonFXConfiguration Config = new TalonFXConfiguration();
        var slot0Configs = Config.Slot0;
        //find values later
        //friction, maybe?
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;

        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(Config);

        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        leftMotor.getConfigurator().apply(Config);


        //Follower follower = new Follower(rightMotor.getDeviceID(), true);
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        

        SmartDashboard.putData("Extend Intake", new InstantCommand(() -> extend()));
        SmartDashboard.putData("Retract Intake", new InstantCommand(() -> retract()));

        

        Mechanism2d mechanism = new Mechanism2d(1.2, 0.6);

        MechanismRoot2d root = mechanism.getRoot("ExtensionRoot", 0.1, 0.3);

        extensionLigament = root.append(new MechanismLigament2d("Extension", 0.0, 0.0)); // horizontal

        SmartDashboard.putData("Extension Mechanism", mechanism);

       
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Position:", getPosition());
    }

    public void simulationPeriodic(){
        double percentExtended = getPosition() / kMaxRotations;

        percentExtended = Math.max(0.0, Math.min(1.0, percentExtended));

        extensionLigament.setLength(percentExtended * kMaxVisualLength);
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
