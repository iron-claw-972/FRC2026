package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

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
    private final DCMotorSim intakeSim;
    private double distance;
    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    public Intake() {
        rightMotor = new TalonFX(IntakeConstants.rightID);
        leftMotor = new TalonFX(IntakeConstants.leftID);

        rollerMotor = new TalonFX(IntakeConstants.rollerID); 

        DCMotor dcmotor = DCMotor.getKrakenX44(2);
     
        intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(2),      // motor model
        0.001,                        // MOI (kg·m²) – tune later
            IntakeConstants.gearRatio
         ),
        DCMotor.getKrakenX44(2)
);
        rightMotor.getSimState().setSupplyVoltage(12.0);

        // get the maximum free speed
        double mechFreeSpeed = Units.radiansToRotations(dcmotor.freeSpeedRadPerSec)/ IntakeConstants.gearRatio;

        // this is confusing
        maxVelocity = 0.5 * mechFreeSpeed;
        maxAcceleration = maxVelocity / 0.25;


        // right motor configs
        TalonFXConfiguration config = new TalonFXConfiguration();
        var slot0Configs = config.Slot0;
        //find values later
        //friction, maybe?
        slot0Configs.kP = 5;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;


        var currentLimits = config.CurrentLimits;
        currentLimits.StatorCurrentLimitEnable = true;
        // set to a low current for testing
        currentLimits.StatorCurrentLimit = 3.0;
        currentLimits.SupplyCurrentLimitEnable = true;
        // set to a low current for testing
        currentLimits.SupplyCurrentLimit = 3.0;

        var motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity =  IntakeConstants.gearRatio * maxVelocity/IntakeConstants.radius/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.gearRatio * maxAcceleration/IntakeConstants.radius/Math.PI/2;

        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.getConfigurator().apply(config);

        leftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        leftMotor.getConfigurator().apply(config);

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
        // Report position to SmartDashboard
        double inchExtension = getPosition();
        SmartDashboard.putNumber("Intake Position:", inchExtension);
        robotExtension.setLength(inchExtension);
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
     * in inches
     * @param setpoint
     */
    public void setPosition(double setpoint) {
        double motorRotations =inchesToRotations(setpoint);
        rightMotor.setControl(voltageRequest.withPosition(motorRotations));
    }

    /**
     * gets motor position in inches
     */
    public double getPosition(){
        // TODO: IDK if this is correct, so check that it's correct!!
        double motorRotations = rightMotor.getPosition().getValueAsDouble();
        return rotationsToInches(motorRotations);

    }

    /**
     * convert rotations to inches
     * @param rotations
     * @return
     */
    public double rotationsToInches(double rotations){
        double teethRackDistance = rotations / (IntakeConstants.rackPitch * Math.PI);
        double positionInches = teethRackDistance/10;
        return positionInches;
    }

    /**
     * convert inches to rotations
     * @param inches
     * @return
     */
    public double inchesToRotations(double inches){
        double teethRackPinion = 10;
        // setpoint is in inches, need to convert to rotations
        double teethRackDistance = inches / (IntakeConstants.rackPitch * Math.PI);
        double rotationsRackPinion = teethRackDistance / teethRackPinion;
        double motorRotations = rotationsRackPinion * IntakeConstants.gearRatio;
        return motorRotations;
    }



    public boolean atMaxExtension(){
        return distance == IntakeConstants.maxExtension; //  TODO add tolerance for distance
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

    public void close() {
        leftMotor.close();
        rightMotor.close();
        rollerMotor.close();
    }

}
    

