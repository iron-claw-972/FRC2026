package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Hood extends SubsystemBase implements HoodIO {
	private TalonFX motor = new TalonFX(IdConstants.HOOD_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

	private double MIN_ANGLE_RAD = Units.degreesToRadians(HoodConstants.MIN_ANGLE);
	private double MAX_ANGLE_RAD = Units.degreesToRadians(HoodConstants.MAX_ANGLE);

	private double MAX_VEL_RAD_PER_SEC = 25;
	private double MAX_ACCEL_RAD_PER_SEC2 = 160.0;

	private double GEAR_RATIO = HoodConstants.HOOD_GEAR_RATIO;

	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02
	, 0.02);

	private double FEEDFORWARD_KV = 0.12;

    //private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0);

	private Rotation2d goalAngle = new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE));
	private double goalVelocityRadPerSec = 0.0;
	private double lastFilteredRad = 0.0;
	private double lastRawSetpoint = 0.0;

	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(0);

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(){
		motor.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
		config.Slot0.kS = 0.1; // Static friction compensation
		config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
		config.Slot0.kD = 0.02; // The "Braking" term to stop overshoot

		var mm = config.MotionMagic;
		mm.MotionMagicCruiseVelocity = Units.radiansToRotations(MAX_VEL_RAD_PER_SEC) * GEAR_RATIO;
		mm.MotionMagicAcceleration = Units.radiansToRotations(MAX_ACCEL_RAD_PER_SEC2) * GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
        motor.getConfigurator().apply(config);

		motor.setPosition(Units.degreesToRotations(HoodConstants.MAX_ANGLE) * GEAR_RATIO);
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	@Override
	public void periodic() {
		updateInputs();
		Logger.processInputs("Hood", inputs);

		double setpointRad = goalAngle.getRadians();

        // calculate shortest angular delta
		double delta = setpointRad - lastRawSetpoint;
		delta = MathUtil.angleModulus(delta);
		
		// filter delta
		double filteredDelta = setpointFilter.calculate(delta);
		
		// apply filtered range
		lastFilteredRad = MathUtil.angleModulus(lastFilteredRad + filteredDelta);
		lastRawSetpoint = setpointRad;
		setpointRad = lastFilteredRad;

		// Tells the Kraken to get to this position using 1000Hz profile
		double motorGoalRotations = Units.radiansToRotations(setpointRad) * GEAR_RATIO;

		//Clamp the setpoint rotations
		motorGoalRotations = MathUtil.clamp(motorGoalRotations, Units.degreesToRotations(HoodConstants.MIN_ANGLE) * GEAR_RATIO, Units.degreesToRotations(HoodConstants.MAX_ANGLE) * GEAR_RATIO);
		
		double velocityCompensation = goalVelocityRadPerSec * FEEDFORWARD_KV;

		motor.setControl(mmVoltageRequest
			.withPosition(motorGoalRotations)
			.withFeedForward(velocityCompensation));

        Logger.recordOutput("Hood/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Hood/velocitySetpoint", goalVelocityRadPerSec / GEAR_RATIO);
		Logger.recordOutput("Hood/SetpointDeg", Units.radiansToDegrees(goalAngle.getRadians()) / GEAR_RATIO);

	}

	@Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / GEAR_RATIO;
		inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
	}
}
