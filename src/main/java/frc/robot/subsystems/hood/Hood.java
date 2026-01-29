package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IdConstants;

public class Hood extends SubsystemBase implements HoodIO {
    final private TalonFX motor;
    private double position;
    private double velocity;
    double power;

    private PIDController pid = new PIDController(0.2, 0.0, 0.05);

    private SingleJointedArmSim hoodSim;
    private static final DCMotor hoodMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(
            Units.degreesToRotations(HoodConstants.START_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
    private double setpoint = HoodConstants.MAX_ANGLE;
    public double distance = HoodConstants.START_DISTANCE;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hoodMotor", 25, 0));

    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood() {
        motor = new TalonFX(IdConstants.HOOD_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

        updateInputs();

        pid.setTolerance(Units.degreesToRadians(3));
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            System.out.println("Simulation in constructor:" + position);
            System.out.println(HoodConstants.START_ANGLE + "Start angle in Hood");
            hoodSim = new SingleJointedArmSim(
                    hoodMotorSim,
                    HoodConstants.HOOD_GEAR_RATIO,
                    0.01 * 0.01 * 5,
                    0.10,
                    Units.degreesToRadians(-360),
                    Units.degreesToRadians(360),
                    false,
                    HoodConstants.START_ANGLE);
        }

        motor.setPosition(Units.degreesToRotations(HoodConstants.START_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

        limitConfig.StatorCurrentLimit = 2; // 120
        limitConfig.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(limitConfig);

        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0.25; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)

        config.Slot0.kP = Units.radiansToRotations(3.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 *
                                                              // 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY)
                * HoodConstants.HOOD_GEAR_RATIO;
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION)
                * HoodConstants.HOOD_GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.getConfigurator().apply(config);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MAX_ANGLE)
                * HoodConstants.HOOD_GEAR_RATIO;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MIN_ANGLE)
                * HoodConstants.HOOD_GEAR_RATIO;

        SmartDashboard.putData("hood", mechanism2d);

        SmartDashboard.putData("Set 45 degrees", new InstantCommand(() -> setSetpoint(45.98)));
        SmartDashboard.putData("Recalibrate Hood", new InstantCommand(() -> resetDueToSlippingError()));

        SmartDashboard.putData("Move to max angle", new InstantCommand(() -> setSetpoint(HoodConstants.MAX_ANGLE)));
        SmartDashboard.putData("Move to min angle", new InstantCommand(() -> setSetpoint(HoodConstants.MIN_ANGLE)));
    }

    public void setSetpoint(double setpoint) {
        double error = MathUtil.inputModulus(setpoint - getPosition(), -180.0, 180.0);
        double shortestDeg = getPosition() + error;
        this.setpoint = shortestDeg;

        double motorTargetRotations = Units
                .degreesToRotations(MathUtil.clamp(shortestDeg, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE))
                * HoodConstants.HOOD_GEAR_RATIO;
        motor.setControl(voltageRequest.withPosition(motorTargetRotations));
    }

    public double getPosition() {
        return inputs.measuredAngle;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getVelocity() {
        return velocity / HoodConstants.HOOD_GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < 3.0;
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        
        //setSetpoint(SmartDashboard.getNumber("hood setpoint", Units.degreesToRadians(getSetpoint())));

        SmartDashboard.putNumber("Hood Position", position);
        SmartDashboard.putNumber("hood setpoint", getSetpoint());

        Logger.recordOutput("HoodPitch", getPosition());

        ligament2d.setAngle(position);

        updateInputs();
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    boolean simInitialized = false;

    @Override
    public void simulationPeriodic() {
        double voltsMotor = motor.getMotorVoltage().getValueAsDouble();
        hoodSim.setInputVoltage(voltsMotor);

        hoodSim.update(Constants.LOOP_TIME);

        double simAngle = hoodSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * HoodConstants.HOOD_GEAR_RATIO;

        encoderSim.setRawRotorPosition(motorRotations);
        encoderSim.setRotorVelocity(
                hoodSim.getVelocityRadPerSec() * Units.radiansToRotations(1) * HoodConstants.HOOD_GEAR_RATIO);
    }

    // Intended to be used for the slipping of the bands that are on the gears
    public void resetDueToSlippingError() {
        while (motor.getSupplyCurrent().getValueAsDouble() < HoodConstants.CURRENT_SPIKE_THRESHHOLD) {
            motor.setVoltage(4);
        }
        position = HoodConstants.START_ANGLE;
    }

    public void updateInputs() {
        inputs.measuredAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())
                / HoodConstants.HOOD_GEAR_RATIO;

        Logger.processInputs("Hood", inputs);
    }
}
