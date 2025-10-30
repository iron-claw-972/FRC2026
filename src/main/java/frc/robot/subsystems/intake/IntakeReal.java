package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeReal extends IntakeBase {
    private TalonFX flyWheelMotor;
    private TalonFX baseMotor;
    private double position;

    private double baseVelocity;
    private double flyWheelVelocity;

    double basePower;
    double flyWheelPower;

    // Increase kG until the arm just holds position against gravity (no PID).
    // Add kS if you notice stiction (motor doesn’t start moving easily).
    // Adjust kV to track moving targets more smoothly.

    private final ArmFeedforward feedforward = new ArmFeedforward(0, IntakeConstants.MASS*IntakeConstants.CENTER_OF_MASS_LENGTH/IntakeConstants.PIVOT_GEAR_RATIO/baseIntakeMotorSim.KtNMPerAmp*baseIntakeMotorSim.rOhms, 0);

    private PIDController pid = new PIDController(0.2, 0.0, 0.05);
    
    private SingleJointedArmSim intakeSim;
    private static final DCMotor baseIntakeMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(IntakeConstants.START_ANGLE * IntakeConstants.PIVOT_GEAR_RATIO);
    private double setpoint = IntakeConstants.START_ANGLE;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("baseMotor", 25, 0));

    public IntakeReal() {
        baseMotor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        flyWheelMotor = new TalonFX(IdConstants.FLYWHEEL_MOTOR_ID);
        pid.setTolerance(Units.degreesToRadians(3));
        encoderSim = baseMotor.getSimState();
        
        intakeSim = new SingleJointedArmSim(
            baseIntakeMotorSim,
            IntakeConstants.PIVOT_GEAR_RATIO,
            //moment of inertia
            IntakeConstants.MOI,
            //length
            IntakeConstants.LENGTH,
            Units.degreesToRadians(0),
            Units.degreesToRadians(360),
            true,
            Units.degreesToRadians(IntakeConstants.START_ANGLE)
        );

        baseMotor.setPosition(Units.degreesToRotations(IntakeConstants.START_ANGLE * IntakeConstants.PIVOT_GEAR_RATIO));
        baseMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = IntakeConstants.MASS * IntakeConstants.CENTER_OF_MASS_LENGTH * 9.8 / IntakeConstants.PIVOT_GEAR_RATIO; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(1.5 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(IntakeConstants.MAX_VELOCITY * IntakeConstants.PIVOT_GEAR_RATIO);
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(IntakeConstants.MAX_ACCELERATION * IntakeConstants.PIVOT_GEAR_RATIO);

        //TODO: Check if this is right
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        baseMotor.getConfigurator().apply(config);

        flyWheelMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        SmartDashboard.putData("intake", mechanism2d);
        SmartDashboard.putData("PID", pid);
        
        SmartDashboard.putData("Set intake stow angle", new InstantCommand(() -> setSetpoint(IntakeConstants.STOW_ANGLE)));
        SmartDashboard.putData("Set intake down angle", new InstantCommand(() -> setSetpoint(IntakeConstants.INTAKE_ANGLE)));
        SmartDashboard.putData("Set intake 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set intake 270 degrees", new InstantCommand(() -> setSetpoint(270)));    
    }

    @Override
    public void setSetpoint(double setpoint) {
        // pid.reset();
        // pid.setSetpoint(Units.degreesToRadians(setPoint));
        //this.setpoint = setpoint;
        baseMotor.setControl(voltageRequest.withPosition(Units.degreesToRotations(setpoint) * IntakeConstants.PIVOT_GEAR_RATIO).withFeedForward(feedforward.calculate(Units.degreesToRadians(position), 0)));
    }

    @Override
    public double getPosition() {
        return position;
    }
    
    @Override
    public double getBaseMotorVelocity() {
        return baseVelocity/IntakeConstants.PIVOT_GEAR_RATIO;
    }

    @Override
    public double getFlyWheelVelocity() {
        return flyWheelVelocity;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < 3.0;
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(baseMotor.getPosition().getValueAsDouble()/ IntakeConstants.PIVOT_GEAR_RATIO);
        baseVelocity = Units.rotationsPerMinuteToRadiansPerSecond(baseMotor.getVelocity().getValueAsDouble() * 60);
        flyWheelVelocity = Units.rotationsPerMinuteToRadiansPerSecond(flyWheelMotor.getVelocity().getValueAsDouble() * 60);

        // double positionRad = Units.degreesToRadians(getPosition());
        // double velocityRadPerSec = baseVelocity;

        // // PID output in volts
        // double pidOutputVolts = pid.calculate(positionRad) * 12.0;

        // // Feedforward voltage (using CURRENT position, not setpoint)
        // double ffVolts = feedforward.calculate(positionRad, velocityRadPerSec);

        // // Combine both
        // double outputVolts = pidOutputVolts + ffVolts;
        // basePower = outputVolts / 12.0;

        //baseMotor.set(basePower);
        flyWheelMotor.set(flyWheelPower);

        ligament2d.setAngle(position);

        // SmartDashboard.putNumber("PID Output (V)", pidOutputVolts);
        // SmartDashboard.putNumber("Feedforward (V)", ffVolts);
        SmartDashboard.putNumber("Total Base Power", basePower);
    }



    public double getAppliedVoltage() {
        return baseMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        //double voltsMotor = basePower * 12;
        double voltsMotor = baseMotor.getMotorVoltage().getValueAsDouble();
        intakeSim.setInputVoltage(voltsMotor);

        intakeSim.update(Constants.LOOP_TIME);

        double simAngle = intakeSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * IntakeConstants.PIVOT_GEAR_RATIO;

        encoderSim.setRawRotorPosition(motorRotations);
    }

    @Override
    public boolean flyWheelSpinning() {
        return flyWheelMotor.getVelocity().getValueAsDouble() > 0;
    }

    @Override
    public void setFlyWheel() {
        flyWheelMotor.set(IntakeConstants.FLYWHEEL_SPEED);
    } 

    @Override
    public void stopFlyWheel(){
        flyWheelMotor.set(0);
    }
}