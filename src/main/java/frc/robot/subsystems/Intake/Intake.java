package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Mechanism Display...
    private final Mechanism2d mechanism;
    private final MechanismLigament2d robotExtension;
    @SuppressWarnings("unused")
    private final MechanismLigament2d robotFrame; 
    private final MechanismLigament2d robotHeight; 
    private final MechanismLigament2d robotPos;

    // create the motors
    /** Motor to move the roller */
    private TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerID, Constants.CANIVORE_SUB);
    /** Right motor (master) */
    private TalonFX rightMotor = new TalonFX(IntakeConstants.rightID, Constants.CANIVORE_SUB);
    /** Left motor (slave) */
    private TalonFX leftMotor = new TalonFX(IntakeConstants.leftID, Constants.CANIVORE_SUB);

    /** Motor characteristics for the roller motor, a Kraken X44 (aka gearbox) */
    private final DCMotor dcMotorRoller = DCMotor.getKrakenX44(1);
    /** Motor characteristics for the extending pair of Kraken X44 motors (aka gearbox) */
    private final DCMotor dcMotorExtend = DCMotor.getKrakenX44(2);

    private double maxVelocity;
    private double maxAcceleration;

    // Use FlywheelSim for the roller
    private FlywheelSim rollerSim;
    // for the moment of inertial, guess .5 kg at a radius of 20 mm
    private double moiRoller = 0.5 * 0.020 * 0.20;
    private double gearingRoller = 2.0;

    // Use ElevatorSim for the extender
    private ElevatorSim intakeSim;

    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    public Intake() {
     
        // get the maximum free speed
        double mechFreeSpeed = Units.radiansToRotations(dcMotorExtend.freeSpeedRadPerSec)/ IntakeConstants.gearRatio;

        // this is confusing
        maxVelocity = 0.5 * mechFreeSpeed;
        maxAcceleration = maxVelocity / 0.25;

        // Configure the motors

        // Build the configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // config the current limits (low value for testing)
        config.CurrentLimits
        .withStatorCurrentLimit(3.0)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(3.0)
        .withSupplyCurrentLimitEnable(true);

        // config Slot 0 PID params
        var slot0Configs = config.Slot0;
        // TODO: set PID parameters
        slot0Configs.kP = 5;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;

        // configure MotionMagic
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity =  IntakeConstants.gearRatio * maxVelocity/IntakeConstants.radius/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.gearRatio * maxAcceleration/IntakeConstants.radius/Math.PI/2;

        // set the brake mode
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // apply the configuration to the right motor (master)
        rightMotor.getConfigurator().apply(config);

        // apply the configuration to the left motor (slave)
        leftMotor.getConfigurator().apply(config);

        // make the left motor follow but oppose the right motor
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Build the mechanism for display
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
        SmartDashboard.putData("Intake On", new InstantCommand(this::spinStart));
        SmartDashboard.putData("Intake Off", new InstantCommand(this::spinStop));

        if (RobotBase.isSimulation()) {
            // build the simulation resources

            // Extender
            // the supply voltage should change with load....
            rightMotor.getSimState().setSupplyVoltage(12.0);

            double carriageMassKg = 3.0;
            double drumRadiusMeters = Units.inchesToMeters(1.0);
            double minHeightMeters = Units.inchesToMeters(0.0);
            double maxHeightMeters = Units.inchesToMeters(IntakeConstants.maxExtension);
            double startingHeightMeters = Units.inchesToMeters(0.0);
            intakeSim = new ElevatorSim(
                dcMotorExtend,
                IntakeConstants.gearRatio,
                carriageMassKg, 
                drumRadiusMeters, 
                minHeightMeters, 
                maxHeightMeters, 
                false, 
                startingHeightMeters);

            // Roller
            rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(dcMotorRoller, moiRoller, gearingRoller),
                dcMotorRoller);
        }
    }

    public void periodic() {
        // Report position to SmartDashboard
        double inchExtension = getPosition();
        SmartDashboard.putNumber("Intake Position:", inchExtension);
        robotExtension.setLength(inchExtension);

        // this returns rotations per second.
        double velocity = rollerMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Roller Velocity", velocity);
    }

    public void simulationPeriodic(){
        // simulate the motor activity

        // get the applied motor voltage
        double voltage = rightMotor.getMotorVoltage().getValueAsDouble();

        // tell the simulator that voltage
        intakeSim.setInputVoltage(voltage);
        // run the siimulation
        intakeSim.update(0.02);

        // get the simulation result
        double metersExtend = intakeSim.getPositionMeters();
        double inchesExtend = Units.metersToInches(metersExtend);
        double motorRotations = inchesToRotations(inchesExtend);

        // set the motor to that position
        rightMotor.getSimState().setRawRotorPosition(motorRotations);

        // update the display
        robotExtension.setLength(inchesExtend);

        // simulate roller
        voltage = rollerMotor.getMotorVoltage().getValueAsDouble();
        rollerSim.setInputVoltage(voltage);
        rollerSim.update(0.020);
        // the X44 has a top speed of 7530 RPM = 125 RPS
        // If the drive is 0.2, then ultimate speed should be 125 RPS * 0.2 = 25 RPS
        // result is 26 RPS.
        double velocity = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()) * gearingRoller;

        rollerMotor.getSimState().setRotorVelocity(velocity);
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
        double motorRotations = rightMotor.getPosition().getValueAsDouble();
        return rotationsToInches(motorRotations);
    }

    /**
     * convert rotations to inches
     * @param rotations
     * @return
     */
    public double rotationsToInches(double motorRotations){
        double circ = 2 * Math.PI * 0.5;
        double pinionRotations = motorRotations / IntakeConstants.gearRatio;
        double inches = pinionRotations * circ;
        return inches; 
    }

    /**
     * convert inches to rotations
     * @param inches
     * @return
     */
    public double inchesToRotations(double inches){
        double circ = 2 * Math.PI * 0.5;
        double pinionRotations = inches / circ;
        double motorRotations = pinionRotations * IntakeConstants.gearRatio;
        return motorRotations;
    }

    public void spin(double speed) {
        rollerMotor.set(speed);
    }

    public void spinStart() {
        spin(IntakeConstants.speed);
    }

    public void spinStop() {
        spin(0.0);
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
