package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
	// Super low magnitude filter for the position to make it less jittery
	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02, 0.02);

	private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

	private TurretIO io;

	public boolean locked = false;

	private boolean calibrating;

	/* ---------------- Hardware ---------------- */

	private TalonFXSimState simState;
	private SingleJointedArmSim turretSim;

	/* ---------------- Control ---------------- */

	private Rotation2d goalAngle = Rotation2d.kZero;
	private double goalVelocityRadPerSec = 0.0;
	private double lastGoalRad = 0.0;
	private double lastFilteredRad = 0.0;
	private double lastRawSetpoint = 0.0;

	/* ---------------- Visualization ---------------- */

	private final Mechanism2d mech = new Mechanism2d(100, 100);
	private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
	private final MechanismLigament2d ligament = root.append(new MechanismLigament2d("barrel", 30, 0));

	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(0);

	/* ---------------- Constructor ---------------- */

	public Turret(TurretIO io) {
		this.io = io;

		lastGoalRad = 0.0;

		if (!Constants.DISABLE_SMART_DASHBOARD) {
			SmartDashboard.putData("Turret Mech", mech);

			SendableChooser<InstantCommand> turretTestChooser = new SendableChooser<>();
			turretTestChooser.setDefaultOption("Turn to 0",
					new InstantCommand(() -> setFieldRelativeTarget(Rotation2d.fromDegrees(0), 0.0)));
			turretTestChooser.addOption("Turn to -90",
					new InstantCommand(() -> setFieldRelativeTarget(Rotation2d.fromDegrees(-90), 0.0)));
			turretTestChooser.addOption("Turn to 90",
					new InstantCommand(() -> setFieldRelativeTarget(Rotation2d.fromDegrees(90), 0.0)));
			turretTestChooser.addOption("Turn to 200",
					new InstantCommand(() -> setFieldRelativeTarget(Rotation2d.fromDegrees(200), 0.0)));
			turretTestChooser.addOption("Turn to -200",
					new InstantCommand(() -> setFieldRelativeTarget(Rotation2d.fromDegrees(-200), 0.0)));

			SmartDashboard.putData("Turret Test Positions", turretTestChooser);
		}
		SmartDashboard.putData("Set Locked", new InstantCommand(() -> {
			locked = !locked;
		}));
		// motor.setPosition(Units.degreesToRotations(238.86) *
		// TurretConstants.GEAR_RATIO);

	}

	/* ---------------- Public API ---------------- */

	/**
	 * Sets the setpoint position and velocity of the turret. Call in command
	 * execute.
	 * 
	 * @param angle
	 * @param velocityRadPerSec
	 */
	public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	/**
	 * @return If the turret is at setpoint with tolerance of 10 degrees
	 */
	public boolean atSetpoint() {
		if (locked)
			return true;
		return Math.abs(goalAngle.getRadians() - getPositionRad()) < Units.degreesToRadians(10.0);
	}

	/**
	 * @return Posiiton of the turret in radians
	 */
	public double getPositionRad() {
		return Units.degreesToRadians(inputs.positionDeg);
	}

	/**
	 * @return Posiiton of the turret in degrees
	 */
	public double getPositionDeg() {
		return inputs.positionDeg;
	}

	/* ---------------- Periodic ---------------- */

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Turret", inputs);

		// Position extrapolation
		double lookAheadSeconds = TurretConstants.EXTRAPOLATION_TIME_CONSTANT;
		double futureRobotAngle = goalAngle.getRadians() + (goalVelocityRadPerSec * lookAheadSeconds);

		// Continuous wrap selection
		double best = lastGoalRad;
		boolean found = false;

		for (int i = -2; i <= 2; i++) {
			double candidate = futureRobotAngle + 2.0 * Math.PI * i;
			if (candidate < Units.degreesToRadians(TurretConstants.MIN_ANGLE)
					|| candidate > Units.degreesToRadians(TurretConstants.MAX_ANGLE))
				continue;

			if (!found || Math.abs(candidate - lastGoalRad) < Math.abs(best - lastGoalRad)) {
				best = candidate;
				found = true;
			}
		}

		lastGoalRad = best;

		// calculate shortest angular delta
		double delta = best - lastRawSetpoint;

		// filter delta
		double filteredDelta = setpointFilter.calculate(delta);

		// apply filtered range
		lastFilteredRad += filteredDelta;
		lastRawSetpoint = best;
		best = lastFilteredRad;

		// Tells the Kraken to get to this position using 1000Hz profile
		double motorGoalRotations = Units.radiansToRotations(best) * TurretConstants.GEAR_RATIO;

		// Clamp position setpoint to min and max angles
		motorGoalRotations = MathUtil.clamp(motorGoalRotations,
				Units.degreesToRotations(TurretConstants.MIN_ANGLE) * TurretConstants.GEAR_RATIO,
				Units.degreesToRotations(TurretConstants.MAX_ANGLE) * TurretConstants.GEAR_RATIO);

		// Multiply goal velocity by kV
		double robotTurnCompensation = goalVelocityRadPerSec * TurretConstants.FEEDFORWARD_KV * TurretConstants.GEAR_RATIO;

		// Sets motor control with feedforward
		io.setControl(mmVoltageRequest
				.withPosition(motorGoalRotations)
				.withFeedForward(robotTurnCompensation)
				.withEnableFOC(true));

		if (!Constants.DISABLE_LOGGING) {
			Logger.recordOutput("Turret/Voltage", inputs.motorVoltage);
			Logger.recordOutput("Turret/setpointDeg", goalAngle.getDegrees());
		}

		// --- Visualization ---
		ligament.setAngle(Units.radiansToDegrees(getPositionRad()));

		if (!Constants.DISABLE_SMART_DASHBOARD) {
			SmartDashboard.putNumber("Turret position", Units.radiansToDegrees(getPositionRad()));
			SmartDashboard.putBoolean("Turret Calibrated", !calibrating);
			SmartDashboard.putBoolean("Turret At Setpoint", atSetpoint());
		}

	}

	/* ---------------- Simulation ---------------- */

	@Override
	public void simulationPeriodic() {
		turretSim.setInputVoltage(inputs.motorVoltage);
		turretSim.update(Constants.LOOP_TIME);

		simState.setRawRotorPosition(
				Units.radiansToRotations(turretSim.getAngleRads()) * TurretConstants.GEAR_RATIO);

		simState.setRotorVelocity(
				Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * TurretConstants.GEAR_RATIO);
	}

	/**
	 * sets supply and stator current limits
	 * 
	 * @param limit the current limit for stator and supply current
	 */
	public void setCurrentLimits(double limit) {
		io.setCurrentLimits(limit);
	}

	// Also ignore this for now
	private double wrapUnit(double value) {
		value %= 1.0;
		if (value < 0)
			value += 1.0;
		return value;
	}
}
