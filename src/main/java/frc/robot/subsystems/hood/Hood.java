package frc.robot.subsystems.hood;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

// TODO: FINNISH (Still work in progress)

public class Hood extends SubsystemBase {
    TalonFX motor = new TalonFX(IdConstants.HOOD_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    double gearRatio = 67/1;

    private double position;
    private double velocity;
    private double power;

    private PIDController pid = new PIDController(0.02, 0, 0);

    //sim
    private SingleJointedArmSim hoodSim;
    private static final DCMotor hoodMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hood_motor", 25, 0));

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(0) * gearRatio);
    private double setpoint = 0;
    Hood() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            hoodSim = new SingleJointedArmSim(hoodMotorSim, gearRatio, HoodConstants.MOI, 0.10, -360, 360, true, 0);
            motor.setPosition(Units.degreesToRotations(0) * gearRatio);
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = 200;
        limitConfig.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(limitConfig);
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0.25; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
        config.Slot0.kP = Units.radiansToRotations(3.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.degreesToRotations(90) * gearRatio; // max 90 degrees in rotations in one second
        motionMagicConfigs.MotionMagicAcceleration = Units.degreesToRotations(90) * gearRatio;
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);
        
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(360) * gearRatio; // max angle * gear ratio
        
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0) * gearRatio; // min angle * gear ratio
        
        SmartDashboard.putData("hood", mechanism2d);
    }
    
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() / gearRatio;
    }
    public boolean atSetPoint() {
        return Math.abs(getPosition() - setpoint) < 3.0;
    }
    // we'll use feed forward and stuff in the command i guess
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        motor.setControl(voltageRequest.withPosition(Units.degreesToRotations(setpoint)));
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / gearRatio;
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);

        ligament2d.setAngle(position);
    }

    @Override
    public void simulationPeriodic() {
        double voltsMotor = getAppliedVoltage();
        hoodSim.setInputVoltage(voltsMotor);

        hoodSim.update(Constants.LOOP_TIME);

        double simAngle = hoodSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * gearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
        encoderSim.setRotorVelocity(hoodSim.getVelocityRadPerSec() * Units.radiansToRotations(1));
    }
}
