package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.util.ChineseRemainderTheorem;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class Turret extends SubsystemBase implements TurretIO{

	/* ---------------- Constants ---------------- */

	private static final double MIN_ANGLE_RAD = Units.degreesToRadians(-180); // Change this later to the actual values
	private static final double MAX_ANGLE_RAD = Units.degreesToRadians(180);

	private static final double MAX_VEL_RAD_PER_SEC = 600;
	private static final double MAX_ACCEL_RAD_PER_SEC2 = 160.0;

	private static final double GEAR_RATIO = TurretConstants.TURRET_GEAR_RATIO;

	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02
	, 0.02);
	//hello

    private final CANcoder encoderLeft = new CANcoder(0, Constants.SUBSYSTEM_CANIVORE_CAN);
    private final CANcoder encoderRight = new CANcoder(1, Constants.SUBSYSTEM_CANIVORE_CAN);

    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

	/* ---------------- Hardware ---------------- */

	private final TalonFX motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

	private TalonFXSimState simState;
	private SingleJointedArmSim turretSim;

	/* ---------------- Control ---------------- */

	private Rotation2d goalAngle = Rotation2d.kZero;
	private double goalVelocityRadPerSec = 0.0;
	private double lastGoalRad = 0.0;
	private State setpoint = new State();
	private double lastFilteredRad = 0.0;
	private double lastRawSetpoint = 0.0;

    // private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0).withUpdateFreqHz(0);

	/* ---------------- Gains ---------------- */

	private static final double kP = 15.0;

	private static final double kD = 0.2;

	/* ---------------- Visualization ---------------- */

	private final Mechanism2d mech = new Mechanism2d(100, 100);
	private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
	private final MechanismLigament2d ligament = root.append(new MechanismLigament2d("barrel", 30, 0));

    // private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0.010);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.1, (1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt) * 0.8, 0);
	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(0);

	/* ---------------- Constructor ---------------- */

	public Turret() {
		motor.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
		config.Slot0.kP = 12.0; 
		config.Slot0.kS = 0.1; // Static friction compensation
		config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
		config.Slot0.kD = 0.15; // The "Braking" term to stop overshoot

		var mm = config.MotionMagic;
		mm.MotionMagicCruiseVelocity = Units.radiansToRotations(MAX_VEL_RAD_PER_SEC) * GEAR_RATIO;
		mm.MotionMagicAcceleration = Units.radiansToRotations(120.0) * GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; //Units.radiansToRotations(400.0) * GEAR_RATIO * 1000000000 * 10000000 * 100000000 * 10000000; // Set to > 0 for "S-Curve" smoothing if needed -- maybe 10-20x the acceleration
        motor.getConfigurator().apply(config);

		setpoint = new State(getPositionRad(), 0.0);
		lastGoalRad = setpoint.position;

		if (RobotBase.isSimulation()) {
			simState = motor.getSimState();
			turretSim = new SingleJointedArmSim(
					edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
					GEAR_RATIO,
					0.01,
					0.15,
					MIN_ANGLE_RAD,
					MAX_ANGLE_RAD,
					false,
					0.0);
		}

		double leftAbs = wrapUnit(encoderLeft.getAbsolutePosition().getValueAsDouble() - TurretConstants.LEFT_ENCODER_OFFSET);
		double rightAbs = wrapUnit(encoderRight.getAbsolutePosition().getValueAsDouble() - TurretConstants.RIGHT_ENCODER_OFFSET);

		int leftTooth = (int) Math.round(leftAbs * TurretConstants.LEFT_ENCODER_TEETH)
				% TurretConstants.LEFT_ENCODER_TEETH;

		int rightTooth = (int) Math.round(rightAbs * TurretConstants.RIGHT_ENCODER_TEETH)
				% TurretConstants.RIGHT_ENCODER_TEETH;

		int turretIndex = ChineseRemainderTheorem.solve(leftTooth, TurretConstants.LEFT_ENCODER_TEETH, rightTooth, TurretConstants.RIGHT_ENCODER_TEETH);

		double totalTeeth = TurretConstants.LEFT_ENCODER_TEETH
        * TurretConstants.RIGHT_ENCODER_TEETH;

		double turretRotations = turretIndex / (double) totalTeeth;

		double motorRotations = turretRotations * TurretConstants.TURRET_GEAR_RATIO;
		motor.setPosition(motorRotations);

		motor.setPosition(0.0); //TODO: remove after hardcrt works

		SmartDashboard.putData("Turret Mech", mech);

		SmartDashboard.putData("Turret to 90", new InstantCommand(()-> setFieldRelativeTarget(new Rotation2d(Math.PI/2), 0.0)));
		SmartDashboard.putData("Turret to -90", new InstantCommand(()-> setFieldRelativeTarget(new Rotation2d(-Math.PI/2), 0.0)));
	}

	/* ---------------- Public API ---------------- */

	public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	public boolean atGoal() {
		return Math.abs(setpoint.position - lastGoalRad) < Units.degreesToRadians(1.5);
	}

	public double getPositionRad() {
		return Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
	}

	/* ---------------- Periodic ---------------- */

	@Override
	public void periodic() {
		updateInputs();
		Logger.processInputs("Turret", inputs);
		
		double robotRelativeGoal = goalAngle.getRadians();

		// --- MA-style continuous wrap selection ---

		double lookAheadSeconds = 0.060; 
    	double futureRobotAngle = goalAngle.getRadians() + (goalVelocityRadPerSec * lookAheadSeconds);

		double best = lastGoalRad;
		boolean found = false;

		for (int i = -2; i <= 2; i++) {
			double candidate = futureRobotAngle + 2.0 * Math.PI * i;
			if (candidate < MIN_ANGLE_RAD || candidate > MAX_ANGLE_RAD)
				continue;

			if (!found || Math.abs(candidate - lastGoalRad) < Math.abs(best - lastGoalRad)) {
				best = candidate;
				found = true;
			}
		}

		lastGoalRad = best;

		// calculate shortest angular delta
		double delta = best - lastRawSetpoint;
		delta = MathUtil.angleModulus(delta);
		
		// filter delta
		double filteredDelta = setpointFilter.calculate(delta);
		
		// apply filtered range
		lastFilteredRad = MathUtil.angleModulus(lastFilteredRad + filteredDelta);
		lastRawSetpoint = best;
		best = lastFilteredRad;

		// Tells the Kraken to get to this position using 1000Hz profile
		double motorGoalRotations = Units.radiansToRotations(best) * GEAR_RATIO;
		// if (isInDeadband) {
		// 	if (absError > START_THRESHOLD_RAD) {
		// 		isInDeadband = false;
		// 		lastGoalRad = best;
		// 	}
		// } else {
		// 	lastGoalRad = best;
		// 	if (absError < STOP_THRESHOLD_RAD) {
		// 		isInDeadband = true;
		// 	}
		// }

		motorGoalRotations = MathUtil.clamp(motorGoalRotations, Units.degreesToRotations(-180) * GEAR_RATIO, Units.degreesToRotations(180) * GEAR_RATIO);
			
		double robotTurnCompensation = goalVelocityRadPerSec * 0.185;

		motor.setControl(mmVoltageRequest
			.withPosition(motorGoalRotations)
			.withFeedForward(robotTurnCompensation));

        Logger.recordOutput("Turret/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Turret/setpointDeg", Units.radiansToDegrees(setpoint.position));

		// --- Visualization ---
		ligament.setAngle(Units.radiansToDegrees(getPositionRad()));

		updateInputs();
		Logger.processInputs("Turret", inputs);

		// SmartDashboard.putNumber("Encoder Left", encoderLeft.get());
		// SmartDashboard.putNumber("Encoder Right", encoderRight.get());
	}

	/* ---------------- Simulation ---------------- */

	@Override
	public void simulationPeriodic() {
		turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
		turretSim.update(Constants.LOOP_TIME);

		simState.setRawRotorPosition(
				Units.radiansToRotations(turretSim.getAngleRads()) * GEAR_RATIO);

		simState.setRotorVelocity(
				Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * GEAR_RATIO);
	}

	@Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / GEAR_RATIO;
		inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.encoderLeftRot = encoderLeft.getAbsolutePosition().getValueAsDouble();
        inputs.encoderRightRot = encoderRight.getAbsolutePosition().getValueAsDouble();
		inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
	}

	private double wrapUnit(double value) {
		value %= 1.0;
		if (value < 0) value += 1.0;
		return value;
	}
}
