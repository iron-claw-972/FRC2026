package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Turret extends SubsystemBase implements TurretIO{

	/* ---------------- Constants ---------------- */

	private static final double MIN_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MIN_ANGLE);
	private static final double MAX_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MAX_ANGLE);

	private static final double MAX_VEL_RAD_PER_SEC = 600;
	private static final double MAX_ACCEL_RAD_PER_SEC2 = 160.0;

	private static final double VERSA_RATIO = 5.0;
	private static final double TURRET_RATIO = 140.0 / 10.0;
	private static final double GEAR_RATIO = VERSA_RATIO * TURRET_RATIO;

	private static final double STOP_THRESHOLD_RAD = Units.degreesToRadians(0.5);  // stop if within 0.5 deg
	private static final double START_THRESHOLD_RAD = Units.degreesToRadians(1.0); // move again if beyond 1.0 deg

	private static final PIDController positionPID = new PIDController(15, 0, 0.25);
	//private static final PIDController longVelocityPID = new PIDController(15, 0, 1.0);
	private static final PIDController velocityPID = new PIDController(0.0, 0.0, 0.0);


    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

	private double lastFrameVelocity = 0.0;

	/* ---------------- Hardware ---------------- */

	private final TalonFX motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.RIO_CAN);

	private TalonFXSimState simState;
	private SingleJointedArmSim turretSim;

	/* ---------------- Control ---------------- */

	private final TrapezoidProfile profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(
					MAX_VEL_RAD_PER_SEC,
					MAX_ACCEL_RAD_PER_SEC2));

	private State setpoint = new State();

	private Rotation2d goalAngle = Rotation2d.kZero;
	private double goalVelocityRadPerSec = 0.0;
	private double lastGoalRad = 0.0;
	private boolean isInDeadband = false;

    // private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0).withUpdateFreqHz(0);

	/* ---------------- Gains ---------------- */

	private static final double kP = 15.0;

	private static final double kD = 0.2;

	private double acclerationAdjustment = 0.0;

	/* ---------------- Visualization ---------------- */

	private final Mechanism2d mech = new Mechanism2d(100, 100);
	private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
	private final MechanismLigament2d ligament = root.append(new MechanismLigament2d("barrel", 30, 0));

    // private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0.010);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * 2.0, 0);
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
		mm.MotionMagicAcceleration = Units.radiansToRotations(180.0) * GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; //Units.radiansToRotations(400.0) * GEAR_RATIO * 1000000000 * 10000000 * 100000000 * 10000000; // Set to > 0 for "S-Curve" smoothing if needed -- maybe 10-20x the acceleration
        motor.getConfigurator().apply(config);

		// Dashboard setup for tuning
        SmartDashboard.putNumber("Turret/kP", config.Slot0.kP);
        SmartDashboard.putNumber("Turret/kS", config.Slot0.kS);
        SmartDashboard.putNumber("Turret/kV", config.Slot0.kV);
        SmartDashboard.putNumber("Turret/kD", config.Slot0.kD);

		// profile = new TrapezoidProfile(new Constraints(MAX_VEL_RAD_PER_SEC, feedForward.maxAchievableAcceleration(DCMotor.getKrakenX60(1, GEAR_RATIO), goalVelocityRadPerSec))))

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

		SmartDashboard.putData("Turret Mech", mech);
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

	/** Updates motor gains if SmartDashboard values change */
    // private void updateTuningGains() {
    //     double p = SmartDashboard.getNumber("Turret/kP", 0);
    //     double s = SmartDashboard.getNumber("Turret/kS", 0);
    //     double v = SmartDashboard.getNumber("Turret/kV", 0);
    //     double d = SmartDashboard.getNumber("Turret/kD", 0);

    //     // Only update if something changed to save CAN bus traffic
    //     if (p != motor.getConfigurator().getConfig(new Slot0Configs()).kP ||
    //         s != motor.getConfigurator().getConfig(new Slot0Configs()).kS) {
            
    //         Slot0Configs slot0 = new Slot0Configs();
    //         slot0.kP = p;
    //         slot0.kS = s;
    //         slot0.kV = v;
    //         slot0.kD = d;
    //         motor.getConfigurator().apply(slot0);
    //     }
    // }

	@Override
	public void periodic() {
		updateInputs();
		Logger.processInputs("Turret", inputs);

		acclerationAdjustment = SmartDashboard.getNumber("Acc Adjust", acclerationAdjustment);
		SmartDashboard.putNumber("Acc Adjust", acclerationAdjustment);
		
		double robotRelativeGoal = goalAngle.getRadians();

		// --- MA-style continuous wrap selection ---

		double lookAheadSeconds = 0.040; 
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

		double currentPositionRad = getPositionRad();
		double errorRad = best - currentPositionRad;
		double absError = Math.abs(errorRad);

		if (isInDeadband) {
			if (absError > START_THRESHOLD_RAD) {
				isInDeadband = false;
				lastGoalRad = best;
			}
		} else {
			lastGoalRad = best;
			if (absError < STOP_THRESHOLD_RAD) {
				isInDeadband = true;
			}
		}

		double motorGoalRotations = Units.radiansToRotations(lastGoalRad) * GEAR_RATIO;

		motorGoalRotations = MathUtil.clamp(motorGoalRotations, Units.degreesToRotations(-180) * GEAR_RATIO, Units.degreesToRotations(180) * GEAR_RATIO);
		
		double acceleration = (goalVelocityRadPerSec - lastFrameVelocity)/Constants.LOOP_TIME;
		// Add the feedforward for the target velocity (SOTM) here as well
		double feedforwardVoltage = feedForward.calculate((goalVelocityRadPerSec) * GEAR_RATIO);
		
		double robotTurnCompensation = goalVelocityRadPerSec * 0.165;

		double feedforward = isInDeadband ? 0.0 : robotTurnCompensation;

		motor.setControl(mmVoltageRequest
			.withPosition(motorGoalRotations)
			.withFeedForward(feedforward));

		Logger.recordOutput("Turret/InDeadband", isInDeadband);

		lastFrameVelocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());

		// var request = velocityRequest.withVelocity(Units.radiansToRotations(targetVelocity)).withEnableFOC(false);
        Logger.recordOutput("Turret/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Turret/setpointDeg", Units.rotationsToDegrees(motorGoalRotations) / GEAR_RATIO);
		//Logger.recordOutput("Turret/velocitySetpoint", targetVelocity / GEAR_RATIO);

		// --- Position + velocity feedforward (MA-style) ---
		// motor.setControl(request);
        // motor.clearStickyFaults();

		// --- Visualization ---
		ligament.setAngle(Units.radiansToDegrees(getPositionRad()));
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
	}
}
