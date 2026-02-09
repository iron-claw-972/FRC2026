package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.InetAddressSerializer;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final Mechanism2d mechanism;
    private final MechanismLigament2d robotExtension;
    private final MechanismLigament2d robotFrame; 
    private final MechanismLigament2d robotHeight; 
    private final MechanismLigament2d robotPos;

    private TalonFX rollerMotor; 
    private TalonFX rightMotor; //leader
    private TalonFX leftMotor; //invert this one
    private double maxVelocity;
    private double maxAcceleration;
    private DCMotorSim intakeSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    public Intake() {
        rightMotor = new TalonFX(IntakeConstants.rightID);
        leftMotor = new TalonFX(IntakeConstants.leftID);
        rollerMotor = new TalonFX(IntakeConstants.rollerID); 
        DCMotor dcmotor = DCMotor.getKrakenX44(2);


        double mechFreeSpeed = 125.0/ IntakeConstants.gearRatio;
        maxVelocity = 0.5 * mechFreeSpeed;
        maxAcceleration = maxVelocity / 0.25;

        // right motor configs
        TalonFXConfiguration Config = new TalonFXConfiguration();
        var slot0Configs = Config.Slot0;
        //find values later
        //friction, maybe?
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;


        var currentLimits = Config.CurrentLimits;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.StatorCurrentLimit = IntakeConstants.statorLimitAmps;
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = IntakeConstants.supplyLimitAmps;

        var motionMagicConfigs = Config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity =  IntakeConstants.gearRatio * maxVelocity/IntakeConstants.radius/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.gearRatio * maxAcceleration/IntakeConstants.radius/Math.PI/2;
        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(Config);

        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        leftMotor.getConfigurator().apply(Config);

        //Follower follower = new Follower(rightMotor.getDeviceID(), true);
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        mechanism = new Mechanism2d(80, 80);
        MechanismRoot2d root = mechanism.getRoot("Root", 0, 1);
        robotPos = root.append(new MechanismLigament2d("robotPos", 40, 0.0, 1, new Color8Bit(0, 0, 0)));
        robotFrame = robotPos.append(new MechanismLigament2d("Robot Frame",28,0.0, 2, new Color8Bit(0, 255, 255)));
        robotHeight = robotPos.append(new MechanismLigament2d("Robot Height", 22.5, 90, 1, new Color8Bit(0,255,255)));
        // extensiion is initially retracted.
        robotExtension = robotHeight.append(new MechanismLigament2d("Robot Extension", 0, 90, 2, new Color8Bit(255, 0, 0) ));

        SmartDashboard.putData("Extension Mechanism", mechanism);
        SmartDashboard.putData("Extend Intake", new InstantCommand(this::extend));
        SmartDashboard.putData("Retract Intake", new InstantCommand(this::retract));
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Position:", getPosition());

        // report the position of the extension
        double percentExtended = getPosition() / IntakeConstants.kMaxRotations;
        double distance = percentExtended/IntakeConstants.gearRatio * 1/IntakeConstants.rackPitch; // in inches
        percentExtended = Math.max(0.0, Math.min(1.0, percentExtended));

        // robotExtension.setLength(percentExtended * maxExtension);

    }

    public void simulationPeriodic(){
        // simulate the motor activity
        double voltage = rightMotor.getMotorVoltage().getValueAsDouble();
        intakeSim.setInputVoltage(voltage);
        intakeSim.update(0.02);
        // rackPitch in teeth/inch
        double mechanicalRotation = intakeSim.getAngularPositionRotations();
        double motorRotation = mechanicalRotation * IntakeConstants.gearRatio; 
        //convert motor rotation to distance
    
    }

    /**
     * in rotations
     * @param setpoint
     */
    public void setPosition(double setpoint) {
        rightMotor.setControl(voltageRequest.withPosition(setpoint));

        // set the position quickly (should be in simultation and move slowly)
        robotExtension.setLength(12.0 * setpoint/IntakeConstants.kMaxRotations);
    }

    /**
     * gets motor position in inches
     */
    public double getPosition(){
        double position = rightMotor.getPosition().getValueAsDouble();
        return position;
    }

    public void spin(double speed) {
        rollerMotor.set(IntakeConstants.speed);
    }

    public void extend() {
       setPosition(IntakeConstants.maxExtension);

    }

    public void retract(){
        setPosition(IntakeConstants.startingPoint);
        
    }

}
    

