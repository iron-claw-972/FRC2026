package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;

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

public class Turret extends SubsystemBase implements TurretIO{
    final private TalonFX motor;
    // Enable here: BUT PROB wont use it
    private boolean infiniteRotation = false;
    private double versaPlanetaryGearRatio = 5.0;
    private double turretGearRatio = 140.0/10.0;
    private final double gearRatio = versaPlanetaryGearRatio * turretGearRatio;
    double power;

    private PIDController pid = new PIDController(0.2, 0.0, 0.05);

    private SingleJointedArmSim turretSim;
    private static final DCMotor turretMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;
    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(0) * gearRatio); // gear ratio
    private double setpoint = 0;
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("turret_motor", 25, 0));

    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret() {
        motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.RIO_CAN); // switch of course
        
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

        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0.0; // Gravity compensation
        config.Slot0.kV = 0.0; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
        config.Slot0.kP = 10.0; // If position error is 1 rotation, apply 10V
        config.Slot0.kI = 0.0; // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = 0.0; // Derivative term (used to dampen oscillations)
        
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(TurretConstants.MAX_VELOCITY / TurretConstants.TURRET_RADIUS) * gearRatio; // max velocity * gear ratio
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(TurretConstants.MAX_ACCELERATION / TurretConstants.TURRET_RADIUS) * gearRatio; // max Acceleration * gear ratio

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        // config.ClosedLoopGeneral.ContinuousWrap = true;

        motor.getConfigurator().apply(config);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(360) * gearRatio; // max angle * gear ratio
        
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0) * gearRatio; // min angle * gear ratio
        
        SmartDashboard.putData("turret", mechanism2d);
        SmartDashboard.putData("PID", pid);

        SmartDashboard.putData("Set to 0 degrees", new InstantCommand(() -> setSetpoint(0, 0)));
        SmartDashboard.putData("Set to 90 degrees", new InstantCommand(( )-> setSetpoint(90, 0)));
        SmartDashboard.putData("Set to 180 degrees", new InstantCommand(() -> setSetpoint(180, 0)));
        SmartDashboard.putData("Set to 270 degrees", new InstantCommand(() -> setSetpoint(270, 0)));

    }

    public double getPosition() {
        return inputs.positionDeg;
    }

    public boolean atSetPoint(){
        return Math.abs(getPosition() - setpoint) < 3.0;
    }

    public void setSetpoint(double setpointDegrees, double robotRotVel) {

        setpoint = MathUtil.clamp(setpointDegrees, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);

        if (infiniteRotation) {
            // Get current position in degrees
            double currentDegrees = (motor.getPosition().getValueAsDouble() / gearRatio) * 360.0;
            // Calculate the error
            double error = setpoint - currentDegrees;
            // Wrap the error to [-180, 180]
            // This finds the "remainder" of the distance relative to a full circle
            double optimizedError = Math.IEEEremainder(error, 360.0);
            // Calculate new target in degrees
            double optimizedSetpointDegrees = currentDegrees + optimizedError;

            double motorTargetRotations = (optimizedSetpointDegrees / 360.0) * gearRatio;
            motor.setControl(voltageRequest.withPosition(motorTargetRotations));
        } else {
            // normal limited 0,360
            double motorTargetRotations = (setpoint / 360.0) * gearRatio;

            //Tune this with rotating robot
            double dV = TurretConstants.ROTATIONAL_VELOCITY_CONSTANT;
            motor.setControl(voltageRequest.withPosition(motorTargetRotations).withFeedForward(dV * robotRotVel));

            System.out.println("Workingnnnnngnggdsfadsfsa");
        }
    }

    @Override
    public void periodic() {        
        updateInputs();
        ligament2d.setAngle(getPosition());

        SmartDashboard.putNumber("Turret Position Degrees", getPosition());        
    }

    @Override
    public void updateInputs(){
        inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / gearRatio;
        inputs.velocity =  Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Turret/SetpointDeg")
    public double getSetpoint(){
        return setpoint;
    }

    @Override
    public void simulationPeriodic() {
        //double voltsMotor = power * 12;
        double voltsMotor = motor.getMotorVoltage().getValueAsDouble();
        turretSim.setInputVoltage(voltsMotor);

        turretSim.update(Constants.LOOP_TIME);

        double simAngle = turretSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * gearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
        encoderSim.setRotorVelocity(turretSim.getVelocityRadPerSec() * Units.radiansToRotations(1) * gearRatio); // Gear Ratio
    }
}