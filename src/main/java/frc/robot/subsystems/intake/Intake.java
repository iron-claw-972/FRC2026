package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Intake extends SubsystemBase{
    private final TalonFX rollerMotor = new TalonFX(0); // TODO
    private final TalonFX pivotMotor = new TalonFX(0, new CANBus()); // TODO

    private SingleJointedArmSim stowArmSim;
    private Mechanism2d stowMechanism2d;
    private MechanismLigament2d stowWheelLigament;

    private final double positionTolerance = 5;

    private final PIDController stowPID = new PIDController(0.015, 0, 0);
    private double power;

    private LaserCan laserCan;
    private boolean hasCoral = false;
    private boolean isMoving = false;
    private Timer laserCanSimTimer;
    private DCMotor dcMotor = DCMotor.getKrakenX60(1);
    private ArmFeedforward feedforward = new ArmFeedforward(0, 3,
            // Constants.GRAVITY_ACCELERATION * IntakeConstants.CENTER_OF_MASS_DIST *
            // IntakeConstants.MASS
            // / IntakeConstants.PIVOT_GEAR_RATIO * dcMotor.rOhms / dcMotor.KtNMPerAmp /
            // Constants.ROBOT_VOLTAGE,
            0); // TODO
    private double startPosition = 90;

    public Intake() {
        if (RobotBase.isSimulation()) {
            stowMechanism2d = new Mechanism2d(10, 10);
            stowWheelLigament = stowMechanism2d.getRoot("Root", 5, 5)
                    .append(new MechanismLigament2d("Intake", 4, startPosition));
            SmartDashboard.putData("Intake pivot", stowMechanism2d);
            stowArmSim = new SingleJointedArmSim(
                    dcMotor,
                    3, // IntakeConstants.PIVOT_GEAR_RATIO,
                    2, // IntakeConstants.MOMENT_OFiNERTIA,
                    1, // IntakeConstants.ARM_LENGTH,
                    Math.toRadians(0),
                    Math.toRadians(90),
                    true,
                    Units.degreesToRadians(startPosition));
            laserCanSimTimer = new Timer();
        } else {
            // laserCan = new LaserCan(IdConstants.INTAKE_LASER_CAN);
            // try {
            // laserCan.setRangingMode(RangingMode.SHORT);
            // laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
            // laserCan.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
            // } catch (ConfigurationFailedException e) {
            // DriverStation.reportError("Intake LaserCan configuration error", true);
            // }
        }
        rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
        pivotMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        pivotMotor.setPosition(Units.degreesToRotations(startPosition) * 1); // TODO replace with IntakeConstants.PIVOT_GEAR_RATIO
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        stowPID.setTolerance(positionTolerance);

        setAngle(startPosition);

        rollerMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));
    }

    @Override
    public void periodic(){
        power = stowPID.calculate(pivotMotor.getPosition().getValueAsDouble());
        power = MathUtil.clamp(power, -1, 1);
        pivotMotor.set(power);
    }

    @Override
    public void simulationPeriodic(){
        stowArmSim.setInputVoltage(power * 1); //TODO change to robo voltsage
        stowArmSim.update(1);//TOOD change smth idk
        stowWheelLigament.setAngle(Units.radiansToDegrees(stowArmSim.getAngleRads()));
    }

    public boolean isAtSetpoint() {
        return stowPID.atSetpoint();
    }


    public PIDController getPidController(){
        return stowPID;
    }

    /**
     * Sets the angle for stowing and unstowing
     * @param position position in degs
     */
    private void setAngle(double position){
        stowPID.setSetpoint(position);
    }

    /**
     * sets the speed
     * @param speed from 0-1
     */
    public void setSpeed(double speed){
        rollerMotor.set(speed);
        isMoving = (Math.abs(speed) < 0.01);
    }

    public void stow(){
        setAngle(90); //TODO set to STOW_SETPOINT constants
    }

    public void unstow(){
        setAngle(0); //TODO set to a constant
    }

    public void stopRollers(){
        setSpeed(0);
    }

    public void startRollers(){
        setSpeed(1); //TODO change to proper motor power
    }

    public boolean laserDetecting(){
        return laserCan.getMeasurement().distance_mm < 100; //TODO add proper constant
    }
}
