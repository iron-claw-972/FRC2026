package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Turret extends SubsystemBase {
    double D_x = 1;
    double D_y = 1;
    final private TalonFX motor;

    private double versaPlanetaryGearRatio = 5.0;
    private double turretGearRatio = 140.0/10.0;
    private final double gearRatio = versaPlanetaryGearRatio * turretGearRatio;

    private double position;
    private double velocity;
    double power;

    private PIDController pid = new PIDController(0.2, 0.0, 0.05);

    private SingleJointedArmSim turretSim;
    private static final DCMotor turretMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;
    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(0) * 1); // 1 is placeholder for gear ratio
    private double setpoint = 0;
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("turret_motor", 25, 0));

    public Turret() {
        motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.CANIVORE_CAN); // switch of course
        
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            turretSim = new SingleJointedArmSim(
                turretMotorSim,
                gearRatio, // gear ratio needs to change
                0.01 * 0.01 * 5,
                0.10,
                Units.degreesToRadians(-360),
                Units.degreesToRadians(360),
                false,
                0.0 // Start angle
            );
        }

        motor.setPosition(Units.degreesToRotations(0) * gearRatio); // gear ratio
        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = 40;
        limitConfig.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(limitConfig);

        // to be frank I just took this from hood because I don't know good values yet
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0.25; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
        config.Slot0.kP = Units.radiansToRotations(3.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)
        
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.degreesToRotations(0.1) * gearRatio; // max velocity * gear ratio
        motionMagicConfigs.MotionMagicAcceleration = Units.degreesToRotations(0.1) * gearRatio; // max Acceleration * gear ratio

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(360) * gearRatio; // max angle * gear ratio
        
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0) * gearRatio; // min angle * gear ratio
        
        SmartDashboard.putData("turret", mechanism2d);
        SmartDashboard.putData("PID", pid);

        SmartDashboard.putData("Set to 1,1", new InstantCommand(() -> setTarget(1,1)));
        SmartDashboard.putData("Set to -1,1", new InstantCommand(( )-> setTarget(-1,1)));
        SmartDashboard.putData("Set to -1,-1", new InstantCommand(() -> setTarget(-1,-1)));
        SmartDashboard.putData("Set to 1,-1", new InstantCommand(() -> setTarget(1,-1)));

        SmartDashboard.putData("Print out target position", new InstantCommand(()-> System.out.println(getTargetPosition())));
    }

    public void setTarget(double x, double y) {
        D_x = x;
        D_y = y;
    }

    public double[] getTargetPosition() {
        System.out.println("Distance X value is: "+ D_x + "and the Distance Y valye is: " + D_y);
        double[] target = {D_x, D_y};
        return target;
    }

    public double getPosition() {
         return motor.getPosition().getValueAsDouble() / gearRatio; // Gear ratio
    }

    public void setSetpoint(double setpointDegrees) {
        // If you want to handle the wrap-around ( -180 to 180)
        double clampedSetpoint = MathUtil.inputModulus(setpointDegrees, -180, 180);
        this.setpoint = clampedSetpoint;
    
        // Convert mechanism degrees to motor rotations: (Degrees / 360) * GearRatio
        double motorTargetRotations = (clampedSetpoint / 360.0) * gearRatio;
        motor.setControl(voltageRequest.withPosition(motorTargetRotations));
    }

    public void align() {
        double angleRad = Math.atan2(D_y, D_x);
        setSetpoint(Units.radiansToDegrees(angleRad));
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / gearRatio; // Gear Ratio
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);

        // SmartDashboard.putNumber("Turret Position", position);
        // align();
        ligament2d.setAngle(position);

    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    boolean simInitialized = false;
    @Override
    public void simulationPeriodic() {
        //double voltsMotor = power * 12;
        double voltsMotor = motor.getMotorVoltage().getValueAsDouble();
        turretSim.setInputVoltage(voltsMotor);

        turretSim.update(Constants.LOOP_TIME);

        double simAngle = turretSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * gearRatio; // gear ratio is 1

        encoderSim.setRawRotorPosition(motorRotations); // MUST set position
        encoderSim.setRotorVelocity(turretSim.getVelocityRadPerSec() * Units.radiansToRotations(1) * gearRatio); // Gear Ratio
    }
}