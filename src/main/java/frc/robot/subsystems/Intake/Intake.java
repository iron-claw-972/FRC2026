package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase implements IntakeIO{
    // Mechanism Display...
    private final Mechanism2d mechanism;
    private final MechanismLigament2d robotExtension;
    @SuppressWarnings("unused")
    private final MechanismLigament2d robotFrame; 
    private final MechanismLigament2d robotHeight; 
    private final MechanismLigament2d robotPos;

    // create the motors
    /** Motor to move the roller */
    private TalonFX rollerMotor = new TalonFX(IdConstants.ROLLER_MOTOR_ID, Constants.CANIVORE_SUB);
    /** Right motor (master) */
    private TalonFX rightMotor = new TalonFX(IdConstants.RIGHT_MOTOR_ID, Constants.CANIVORE_SUB);
    /** Left motor (slave) */
    private TalonFX leftMotor = new TalonFX(IdConstants.LEFT_MOTOR_ID, Constants.CANIVORE_SUB);

    /** Motor characteristics for the roller motor, a single Kraken X44 (aka gearbox) */
    private final DCMotor dcMotorRoller = DCMotor.getKrakenX44(1);
    /** Motor characteristics for the extending pair of Kraken X44 motors (aka gearbox) */
    private final DCMotor dcMotorExtend = DCMotor.getKrakenX44(2);

    private double maxVelocity;
    private double maxAcceleration;

    // Use FlywheelSim for the roller
    private FlywheelSim rollerSim;

    // Use ElevatorSim for the extender
    private ElevatorSim intakeSim;

    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    private double setpointInches = 0.0;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake() {
     
        // get the maximum free speed
        double maxFreeSpeed = Units.radiansToRotations(dcMotorExtend.freeSpeedRadPerSec)/ IntakeConstants.GEAR_RATIO;

        // max free speed (rot/s) = motor free speed (rad/s to rot/s)/ gear ratio
        // safety margin, limits velocity to .75 free speed
        maxVelocity = 0.75 * maxFreeSpeed;
        maxAcceleration = maxVelocity / 0.25;

        // Configure the motors
        // Build the configuration for the roller
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        // config Slot 0 PID params
        var slot0Configs = rollerConfig.Slot0;
        slot0Configs.kP = 5.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;

        // set the brake mode
        rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // apply the configuration to the right motor (master)
        rollerMotor.getConfigurator().apply(rollerConfig);

        // Build the configuration for the left and right Motor
        TalonFXConfiguration config = new TalonFXConfiguration();

        // config the current limits (low value for testing)
        config.CurrentLimits
        .withStatorCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMITS)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMITS)
        .withSupplyCurrentLimitEnable(true);

        // config Slot 0 PID params
        var rollerSlot0Configs = config.Slot0;
        rollerSlot0Configs.kP = 5.0;
        rollerSlot0Configs.kI = 0.0;
        rollerSlot0Configs.kD = 0.0;
        rollerSlot0Configs.kV = 0.0;
        rollerSlot0Configs.kA = 0.0;

        // configure MotionMagic
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity =  IntakeConstants.GEAR_RATIO * maxVelocity/IntakeConstants.RADIUS_RACK_PINION/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.GEAR_RATIO * maxAcceleration/IntakeConstants.RADIUS_RACK_PINION/Math.PI/2;

        rightMotor.getConfigurator().apply(config);
        leftMotor.getConfigurator().apply(config);

        leftMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        rightMotor.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);

        // Build the mechanism for display
        mechanism = new Mechanism2d(80, 80);
        MechanismRoot2d root = mechanism.getRoot("Root", 0, 1);
        robotPos = root.append(new MechanismLigament2d("robotPos", 40, 0.0, 1, new Color8Bit(0, 0, 0)));
        robotFrame = robotPos.append(new MechanismLigament2d("Robot Frame",28,0.0, 2, new Color8Bit(0, 255, 255)));
        robotHeight = robotPos.append(new MechanismLigament2d("Robot Height", 22.5, 90, 1, new Color8Bit(0,255,255)));
        // extensiion is initially retracted.
        robotExtension = robotHeight.append(new MechanismLigament2d("Robot Extension", 0, 90, 2, new Color8Bit(255, 0, 0) ));

        // add some test commands.
        SmartDashboard.putData("Extension Mechanism", mechanism);
        SmartDashboard.putData("START INTAKE COMMAND", new InstantCommand(()->{
            extend();
            spinStart();
        }));
        SmartDashboard.putData("END INTAKE COMMAND", new InstantCommand(()->{
            intermediateExtend();
            spinStop();
        }));

        if (RobotBase.isSimulation()) {
            // Extender simulation
            // the supply voltage should change with load....
            rightMotor.getSimState().setSupplyVoltage(12.0);

            // rack pinion is 10 teeth and 10 DP for a radius of 1 inches
            double drumRadiusMeters = Units.inchesToMeters(1.0);
            double minHeightMeters = Units.inchesToMeters(0.0);
            double maxHeightMeters = Units.inchesToMeters(IntakeConstants.MAX_EXTENSION);
            // start retracted
            double startingHeightMeters = Units.inchesToMeters(0.0);
            intakeSim = new ElevatorSim(
                dcMotorExtend,
                IntakeConstants.GEAR_RATIO,
                IntakeConstants.CARRIAGE_MASS_KG, 
                drumRadiusMeters, 
                minHeightMeters, 
                maxHeightMeters, 
                false, 
                startingHeightMeters);

            // Roller simulation
            rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(dcMotorRoller, IntakeConstants.ROLLER_MOI_KG_M_SQ, IntakeConstants.ROLLER_GEARING),
                dcMotorRoller);
        }
    }

    public void periodic() {
        // Report position to SmartDashboard
        double inchExtension = getPosition();
        Logger.recordOutput("Intake/Setpoint", setpointInches);
        robotExtension.setLength(inchExtension);

        // Report velocity to SmartDashbboard
        // this returns rotations per second.
        double velocity = rollerMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Roller Velocity", velocity);

        updateInputs();
        Logger.processInputs("Intake", inputs);
    }

    public void simulationPeriodic(){
        // simulate the motor activities

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

        double velocity = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()) * IntakeConstants.ROLLER_GEARING;

        rollerMotor.getSimState().setRotorVelocity(velocity);
    }

    /**
     * Set the intake extender position
     * @param setpoint in inches
     */
    public void setPosition(double setpoint) {
        double motorRotations = inchesToRotations(setpoint);
        rightMotor.setControl(voltageRequest.withPosition(motorRotations));
        leftMotor.setControl(voltageRequest.withPosition(motorRotations));

        setpointInches = setpoint;
    }

    /**
     * Get the intake extender position
     * @return inches
     */
    public double getPosition(){
        return inputs.leftPosition;
    }

    /**
     * convert rotations to inches
     * @param rotations of the motor
     * @return inches of rack travel
     */
    public double rotationsToInches(double motorRotations) {
        // circumference of the rack pinion
        double circ = 2 * Math.PI * 0.5;
        double pinionRotations = motorRotations / IntakeConstants.GEAR_RATIO;
        double inches = pinionRotations * circ;
        return inches; 
    }

    /**
     * convert inches to rotations
     * @param inches of rack travel
     * @return motor rotations
     */
    public double inchesToRotations(double inches){
        double circ = 2 * Math.PI * 0.5;
        double pinionRotations = inches / circ;
        double motorRotations = pinionRotations * IntakeConstants.GEAR_RATIO;
        return motorRotations;
    }

    /**
     * Set the roller speed.
     * @param speed duty cycle in the range [-1, 1]
     */
    public void spin(double speed) {
        rollerMotor.set(speed);
    }

    /**
     * Start the intake roller spinning.
     */
    public void spinStart() {
        spin(IntakeConstants.SPEED);
    }

    /**
     * Stop the intake roller.
     */
    public void spinStop() {
        spin(0.0);
    }

    /**
     * Reverses the intake roller
     */
    public void spinReverse(){
        spin(-IntakeConstants.SPEED);
    }

    /** Extend the intake the maximum distance. */
    public void extend() {
       setPosition(IntakeConstants.MAX_EXTENSION);
    }

    /** Extend to a position that doesn't hit the spindexer */
    public void intermediateExtend(){
        setPosition(IntakeConstants.INTERMEDIATE_EXTENSION);
    }

    /** Retract the intake to a safe starting position. */
    public void retract(){
        setPosition(IntakeConstants.STOW_EXTENSION);
    }

    /** Goes to the zero position */
    public void zeroPosition(){
        setPosition(0.0);
    }

    public void zeroMotors() {
        rightMotor.setPosition(0.0);
        leftMotor.setPosition(0.0);
    }

    /**
     * Reclaim all the resources (e.g., motors and other devices).
     * This step is necessary for multiple unit tests to work.
     */
    public void close() {
        leftMotor.close();
        rightMotor.close();
        rollerMotor.close();
    }

    public void calibrate(){
        
    }

    @Override
    public void updateInputs() {
        inputs.leftPosition = rotationsToInches(leftMotor.getPosition().getValueAsDouble());
        inputs.rightPosition = rotationsToInches(rightMotor.getPosition().getValueAsDouble());
        inputs.leftCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    }

}
