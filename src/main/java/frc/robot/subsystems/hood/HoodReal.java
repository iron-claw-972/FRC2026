package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IdConstants;

public class HoodReal extends HoodBase {
    final private TalonFX motor;
    private double position;
    private double velocity;
    double power;
    
    private PIDController pid = new PIDController(0.2, 0.0, 0.05);
    //TODO: get actual gear ratio
    private double hoodGearRatio = 50.0;

    private SingleJointedArmSim hoodSim;
    private static final DCMotor hoodMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(HoodConstants.START_ANGLE*hoodGearRatio);
    private double setpoint = HoodConstants.START_ANGLE;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hoodMotor", 25, 0));

    public HoodReal() {
        // allocate the motor
        motor = new TalonFX(IdConstants.HOOD_MOTOR_ID);

        pid.setTolerance(Units.degreesToRadians(3));

        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();

            hoodSim = new SingleJointedArmSim(
                hoodMotorSim,
                hoodGearRatio,
                0.01 * 0.01 * 5,
                0.10,
                Units.degreesToRadians(-360),
                Units.degreesToRadians(360),
                false,
                Units.degreesToRadians(HoodConstants.START_ANGLE)
            );
        }

        // motor position at power up
        motor.setPosition(Units.degreesToRotations(HoodConstants.START_ANGLE * hoodGearRatio));
        motor.setNeutralMode(NeutralModeValue.Brake);


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(0.8 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY * hoodGearRatio);
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION * hoodGearRatio);
        //TODO: find which direction is positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        motor.getConfigurator().apply(config);

        SmartDashboard.putData("hood", mechanism2d);
        SmartDashboard.putData("PID", pid);
        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(270)));
    }

    public void setSetpoint(double setpoint) {
        // pid.reset();
        // pid.setSetpoint(Units.degreesToRadians(setpoint));
        this.setpoint = setpoint;
        motor.setControl(voltageRequest.withPosition(Units.degreesToRotations(setpoint) * hoodGearRatio));
    }

    public double getPosition() {
        return position;
    }
    
    public double getVelocity() {
        return velocity/hoodGearRatio;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < 3.0;
    }

    @Override
    public void periodic() {
        //try find a way to do with motion magic
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / hoodGearRatio;
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60) / hoodGearRatio;
        //power = pid.calculate(Units.degreesToRadians(getPosition()));
        //motor.set(power);
        
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
        hoodSim.setInputVoltage(voltsMotor);
        System.out.println(voltsMotor);

        hoodSim.update(Constants.LOOP_TIME);

        double simAngle = hoodSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * hoodGearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
    }
}