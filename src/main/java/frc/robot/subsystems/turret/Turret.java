package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.constants.VisionConstants;

public class Turret extends SubsystemBase implements TurretIO{

	/* ---------------- Constants ---------------- */

	private static final double MIN_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MIN_ANGLE);
	private static final double MAX_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MAX_ANGLE);

	private static final double MAX_VEL_RAD_PER_SEC = 25;
	private static final double MAX_ACCEL_RAD_PER_SEC2 = 160.0;

	private static final double VERSA_RATIO = 5.0;
	private static final double TURRET_RATIO = 140.0 / 10.0;
	private static final double GEAR_RATIO = VERSA_RATIO * TURRET_RATIO;

	private static final PIDController positionPID = new PIDController(15, 0, 0.25);
	//private static final PIDController longVelocityPID = new PIDController(15, 0, 1.0);
	private static final PIDController velocityPID = new PIDController(0.0, 0.0, 0.0);


    private final TurretIOInputs inputs = new TurretIOInputs();

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

    // private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0).withUpdateFreqHz(0);

	/* ---------------- Gains ---------------- */

	private static final double kP = 15.0;

	private static final double kD = 0.2;

	/* ---------------- Visualization ---------------- */

	private final Mechanism2d mech = new Mechanism2d(100, 100);
	private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
	private final MechanismLigament2d ligament = root.append(new MechanismLigament2d("barrel", 30, 0));

    // private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0.010);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0);


	/* ---------------- Constructor ---------------- */

	public Turret() {
		motor.setNeutralMode(NeutralModeValue.Coast);

		motor.getConfigurator().apply(
				new Slot0Configs()
						.withKP(kP)
						.withKD(kD));

		TalonFXConfiguration config = new TalonFXConfiguration();
		var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10 * GEAR_RATIO;
        motionMagicConfigs.MotionMagicAcceleration = 50 * GEAR_RATIO;
        motor.getConfigurator().apply(config);

		motor.getConfigurator().apply(
				new com.ctre.phoenix6.configs.TalonFXConfiguration() {
					{
						MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
					}
				});

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

	public java.util.function.Supplier<Transform3d> getCameraLocationSupplier() {
		return () -> {
			double turretAngle = getPositionRad();
			Translation3d translation = new Translation3d(
					VisionConstants.TURRET_CAMERA_X_OFFSET,
					VisionConstants.TURRET_CAMERA_Y_OFFSET,
					VisionConstants.TURRET_CAMERA_Z_OFFSET);
			Rotation3d rotation = new Rotation3d(
					0,
					VisionConstants.TURRET_CAMERA_PITCH,
					turretAngle + VisionConstants.TURRET_CAMERA_ROLL);
			return new Transform3d(translation, rotation);
		};
	}

	/* ---------------- Periodic ---------------- */

	@Override
	public void periodic() {
		updateInputs();
		
		double robotRelativeGoal = goalAngle.getRadians();

		// --- MA-style continuous wrap selection ---
		double best = lastGoalRad;
		boolean found = false;

		for (int i = -2; i <= 2; i++) {
			double candidate = robotRelativeGoal + 2.0 * Math.PI * i;
			if (candidate < MIN_ANGLE_RAD || candidate > MAX_ANGLE_RAD)
				continue;

			if (!found || Math.abs(candidate - lastGoalRad) < Math.abs(best - lastGoalRad)) {
				best = candidate;
				found = true;
			}
		}

		lastGoalRad = best;

		// --- Profile in MECHANISM SPACE ---
		State goalState = new State(
				MathUtil.clamp(best, MIN_ANGLE_RAD, MAX_ANGLE_RAD),
				goalVelocityRadPerSec);

		setpoint = profile.calculate(
				Constants.LOOP_TIME,
				setpoint,
				goalState);

		// --- Convert to MOTOR SPACE ---
		double motorPosRot = Units.radiansToRotations(setpoint.position) * GEAR_RATIO;

		double motorVelRotPerSec = Units.radiansToRotations(setpoint.velocity) * GEAR_RATIO;

		double targetVelocity;

		// if(Math.abs(setpoint.position * GEAR_RATIO - Units.rotationsToRadians(motor.getPosition().getValueAsDouble())) > Math.PI/2){
		// 	// in rad/sec
		// 	targetVelocity = longVelocityPID.calculate(
		// 			motor.getPosition().getValue().in(edu.wpi.first.units.Units.Radians),
		// 			setpoint.position * GEAR_RATIO);
			
		// 	targetVelocity += Units.rotationsToRadians(motorVelRotPerSec);

		// 	double voltage = feedForward.calculate(targetVelocity);
		// 	motor.setVoltage(voltage);
		// } else{
			// in rad/sec
			//double robotRotAcceleration = (Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) - lastFrameVelocity) / Constants.LOOP_TIME;
			double motorSetpointPosition = (setpoint.position) * GEAR_RATIO;

			targetVelocity = positionPID.calculate(
					motor.getPosition().getValue().in(edu.wpi.first.units.Units.Radians),
					motorSetpointPosition);
			
			targetVelocity += Units.rotationsToRadians(motorVelRotPerSec);

			double voltage = feedForward.calculate(targetVelocity);

			double velocityCorrectionVoltage = velocityPID.calculate(Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()), targetVelocity);
			voltage += velocityCorrectionVoltage;

			motor.setVoltage(voltage);
		// }
		lastFrameVelocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());

		// var request = velocityRequest.withVelocity(Units.radiansToRotations(targetVelocity)).withEnableFOC(false);
        Logger.recordOutput("Turret/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Turret/velocitySetpoint", targetVelocity / GEAR_RATIO);

		// --- Position + velocity feedforward (MA-style) ---
		// motor.setControl(request);
        // motor.clearStickyFaults();

		// --- Visualization ---
		ligament.setAngle(Units.radiansToDegrees(getPositionRad()));

        SmartDashboard.putData(positionPID);
		SmartDashboard.putData(velocityPID);
		SmartDashboard.putNumber("Turret GoalDeg",
				Units.radiansToDegrees(best));
		SmartDashboard.putNumber("Turret SetpointDeg",
				Units.radiansToDegrees(setpoint.position));
		SmartDashboard.putNumber("Turret Raw Setpoint", Units.radiansToDegrees(best));
		SmartDashboard.putNumber("Turret motorPosRot",
				Units.radiansToDegrees(motorPosRot));
		SmartDashboard.putNumber("Turret motorVelRotPerSec",
				Units.radiansToDegrees(motorVelRotPerSec));
        SmartDashboard.putNumber("Turret targetVelocity",
				Units.radiansToDegrees(targetVelocity));
		SmartDashboard.putNumber("Turret Position Deg",
				Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / GEAR_RATIO);
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
