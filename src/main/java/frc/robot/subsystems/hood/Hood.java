package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Hood extends SubsystemBase {

	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02, 0.02);

	private Rotation2d goalAngle = new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE));
	private double goalVelocityRadPerSec = 0.0;
	private double lastFilteredRad = 0.0;
	private double lastRawSetpoint = 0.0;

	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(
			Units.degreesToRotations(HoodConstants.MAX_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);

	private boolean calibrating = false;
	private Debouncer calibrateDebouncer = new Debouncer(0.5, DebounceType.kRising);

	private boolean forceHoodDown = false;

	private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
	private HoodIO io;

	public Hood(HoodIO io) {
		this.io = io;

		if (!Constants.DISABLE_SMART_DASHBOARD) {
			SmartDashboard.putData("max", new InstantCommand(
					() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0)));
			SmartDashboard.putData("medium", new InstantCommand(() -> setFieldRelativeTarget(
					new Rotation2d(Units.degreesToRadians((HoodConstants.MAX_ANGLE + HoodConstants.MIN_ANGLE) / 2)), 0)));
			SmartDashboard.putData("min", new InstantCommand(
					() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MIN_ANGLE)), 0)));

			SmartDashboard.putData("force hood down", new InstantCommand(() -> forceHoodDown(true)));
			SmartDashboard.putData("unforce hood", new InstantCommand(() -> forceHoodDown(false)));
		}
	}

	/**
	 * @return Position of the MOTOR in radians
	 */
	public double getMotorPositionRad() {
		return Units.degreesToRadians(inputs.positionDeg);
	}

	/**
	 * Sets the setpoint position and velocity of the hood. Call in command execute.
	 * 
	 * @param angle
	 * @param velocityRadPerSec
	 */
	public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	/**
	 * @return Position of turret in degrees
	 */
	public double getPositionDeg() {
		return inputs.positionDeg;
	}

	public void forceHoodDown(boolean taranNathan) {
		forceHoodDown = taranNathan;
	}

	public boolean getHoodForcedDown() {
		return this.forceHoodDown;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);

		// goalAngle = Rotation2d.fromDegrees(SmartDashboard.getNumber("Hood Setpoint",
		// goalAngle.getDegrees()));
		// SmartDashboard.putNumber("Hood Setpoint", goalAngle.getDegrees());

		if (forceHoodDown) {
			goalAngle = Rotation2d.fromDegrees(HoodConstants.MAX_ANGLE);
			goalVelocityRadPerSec = 0.0;
		}


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
		double motorGoalRotations = Units.radiansToRotations(setpointRad) * HoodConstants.HOOD_GEAR_RATIO;

		// Clamp the setpoint rotations
		motorGoalRotations = MathUtil.clamp(motorGoalRotations,
				Units.radiansToRotations(Units.degreesToRadians(HoodConstants.MIN_ANGLE)) * HoodConstants.HOOD_GEAR_RATIO,
				Units.radiansToRotations(Units.degreesToRadians(HoodConstants.MAX_ANGLE)) * HoodConstants.HOOD_GEAR_RATIO);

		// Multiply goal velocity by kV
		double velocityCompensation = goalVelocityRadPerSec * HoodConstants.FEEDFORWARD_KV;

		if (calibrating) {
			io.setMotorRaw(0.1);
			boolean atZero = Math
					.abs(inputs.motorCurrent) >= HoodConstants.CALIBRATION_CURRENT_THRESHOLD;
			boolean calibrated = calibrateDebouncer.calculate(atZero);
		} else {
			// Set control with feedforward
			io.setMotorControl(mmVoltageRequest
					.withPosition(motorGoalRotations)
					.withFeedForward(velocityCompensation)
					.withEnableFOC(true));
		}

		if (!Constants.DISABLE_LOGGING) {
			Logger.recordOutput("Hood/Voltage", inputs.motorVoltage);
			Logger.recordOutput("Hood/velocitySetpoint", goalVelocityRadPerSec / HoodConstants.HOOD_GEAR_RATIO);
			Logger.recordOutput("Hood/SetpointDeg", Units.radiansToDegrees(goalAngle.getRadians()));
		}
	}

	/**
	 * sets supply and stator current limits
	 * 
	 * @param limitAmps the current limit for stator and supply current
	 */
	public void setCurrentLimits(double stator, double supply) {
		io.setCurrentLimits(stator, supply);
	}
}
