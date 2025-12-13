package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import frc.robot.constants.IntakeConstants;

public class IntakeReal extends SubsystemBase implements IntakeIO{
    private TalonFX flyWheelMotor = new TalonFX(IdConstants.FLYWHEEL_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX baseMotor = new TalonFX(IdConstants.BASE_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

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

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0.0 * IntakeConstants.PIVOT_GEAR_RATIO);
    private double setpoint;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("baseMotor", 25, 0));

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IdConstants.INTAKE_ENCODER_ID);

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeReal() {
        updateInputs();
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

        double absoluteAngleDegrees =  getAbsoluteEncoderAngle() - IntakeConstants.ABSOLUTE_OFFSET_ANGLE;
        baseMotor.setPosition(Units.degreesToRotations(absoluteAngleDegrees * IntakeConstants.PIVOT_GEAR_RATIO));
        baseMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = IntakeConstants.MASS * IntakeConstants.CENTER_OF_MASS_LENGTH * 9.8 / IntakeConstants.PIVOT_GEAR_RATIO; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(0.8 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(IntakeConstants.MAX_VELOCITY * IntakeConstants.PIVOT_GEAR_RATIO);
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(IntakeConstants.MAX_ACCELERATION * IntakeConstants.PIVOT_GEAR_RATIO);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        baseMotor.getConfigurator().apply(config);

         CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

        limitConfig.StatorCurrentLimit = 30; // 120
        limitConfig.StatorCurrentLimitEnable = true;

        baseMotor.getConfigurator().apply(limitConfig);

        flyWheelMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );
        
        SmartDashboard.putData("Set intake stow angle", new InstantCommand(() -> setSetpoint(IntakeConstants.STOW_ANGLE)));
        SmartDashboard.putData("Set intake down angle", new InstantCommand(() -> setSetpoint(IntakeConstants.INTAKE_ANGLE)));
        SmartDashboard.putData("Set intake 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set intake 270 degrees", new InstantCommand(() -> setSetpoint(270)));    
    
        SmartDashboard.putData("Spin flywheel motor", new InstantCommand(() -> setFlyWheel()));
        SmartDashboard.putData("Stop flywheel motor", new InstantCommand(() -> stopFlyWheel()));
    
        SmartDashboard.putData("Get absolute encoder angle", new InstantCommand(() -> System.out.println(getAbsoluteEncoderAngle())));
        SmartDashboard.putNumber("Absolute Encoder Value", getAbsoluteEncoderAngle());
    }

    public void setSetpoint(double setpoint) {
        double clampedSetpoint = MathUtil.clamp(setpoint, 90.0, 180.0);
        baseMotor.setControl(voltageRequest.withPosition(Units.degreesToRotations(clampedSetpoint) * IntakeConstants.PIVOT_GEAR_RATIO).withFeedForward(feedforward.calculate(Units.degreesToRadians(getAngle()), 0)));
    }

    @AutoLogOutput
    public double getAbsoluteEncoderAngle() {
        double rotations = absoluteEncoder.get();
        double armRotations = rotations / (IntakeConstants.PIVOT_GEAR_RATIO / 18.0);
        return Units.rotationsToDegrees(armRotations);
    }

    /**
     * Gets the angle of the intake
     * @return The angle in degrees
     */
    public double getAngle() {
        // if(RobotBase.isSimulation()){
        //     return position;
        // }
        return inputs.measuredAngle;
    }

    public double getFlyWheelVelocity() {
        return inputs.flyWheelVelocity;
    }

    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpoint) < 3.0;
    }

    @Override
    public void periodic() {
        updateInputs();

        flyWheelMotor.set(flyWheelPower);

        ligament2d.setAngle(getAngle());

        SmartDashboard.putNumber("Total Base Power", basePower);
    }

    public double getAppliedVoltage() {
        return baseMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        double voltsMotor = baseMotor.getMotorVoltage().getValueAsDouble();
        intakeSim.setInputVoltage(voltsMotor);

        intakeSim.update(Constants.LOOP_TIME);

        double simAngle = intakeSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * IntakeConstants.PIVOT_GEAR_RATIO;

        encoderSim.setRawRotorPosition(motorRotations);
    }

    public boolean flyWheelSpinning() {
        return flyWheelMotor.getVelocity().getValueAsDouble() > 0;
    }

    public void setFlyWheel() {
        flyWheelPower = IntakeConstants.FLYWHEEL_SPEED;
    }

    public void stopFlyWheel(){
        flyWheelPower = 0;
    }

    @AutoLogOutput
    public double setpointAngle(){
        return setpoint;
    }

    @Override
    public void updateInputs() {
        inputs.measuredAngle = Units.rotationsToDegrees(baseMotor.getPosition().getValueAsDouble()/ IntakeConstants.PIVOT_GEAR_RATIO);
        inputs.currentAmps = baseMotor.getStatorCurrent().getValueAsDouble();
        inputs.flyWheelVelocity = Units.rotationsPerMinuteToRadiansPerSecond(flyWheelMotor.getVelocity().getValueAsDouble() * 60);

        Logger.processInputs("Intake", inputs);
    }
}