package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.constants.IdConstants;

public class Hood extends SubsystemBase implements HoodIO{
    private TalonFX motor = new TalonFX(IdConstants.HOOD_ID, Constants.CANIVORE_SUB);

	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02, 0.02);

	private Rotation2d goalAngle = new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE));
	private double goalVelocityRadPerSec = 0.0;
	private double lastFilteredRad = 0.0;
	private double lastRawSetpoint = 0.0;

	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(0);

	private boolean calibrating = false;
	private Debouncer calibrateDebouncer = new Debouncer(0.5, DebounceType.kRising);

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(){
		motor.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
		config.Slot0.kP = 2.0; 
		config.Slot0.kS = 0.1; // Static friction compensation
		config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
		config.Slot0.kD = 0.02; // The "Braking" term to stop overshoot

		var mm = config.MotionMagic;
		mm.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY) * HoodConstants.HOOD_GEAR_RATIO;
		mm.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION) * HoodConstants.HOOD_GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
        motor.getConfigurator().apply(config);

		setCurrentLimits(HoodConstants.NORMAL_CURRENT_LIMIT);

		motor.setPosition(Units.degreesToRotations(HoodConstants.MAX_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);

		SmartDashboard.putData("max", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0)));
		SmartDashboard.putData("medium", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians((HoodConstants.MAX_ANGLE + HoodConstants.MIN_ANGLE) / 2)), 0)));
		SmartDashboard.putData("min", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MIN_ANGLE)), 0)));
    }

	/**
	 * @return Position of the MOTOR in radians
	 */
    public double getMotorPositionRad(){
        return Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    }

	/**
	 * Sets the setpoint position and velocity of the hood. Call in command execute.
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
	public double getPositionDeg(){
		return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
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
		double motorGoalRotations = Units.radiansToRotations(setpointRad) * HoodConstants.HOOD_GEAR_RATIO;

		//Clamp the setpoint rotations
		motorGoalRotations = MathUtil.clamp(motorGoalRotations, Units.radiansToRotations(Units.degreesToRadians(HoodConstants.MIN_ANGLE)) * HoodConstants.HOOD_GEAR_RATIO, Units.radiansToRotations(Units.degreesToRadians(HoodConstants.MAX_ANGLE)) * HoodConstants.HOOD_GEAR_RATIO);
		
		// Multiply goal velocity by kV
		double velocityCompensation = goalVelocityRadPerSec * HoodConstants.FEEDFORWARD_KV;

		if (calibrating){
			motor.set(0.1);
		} else{
			// Set control with feedforward
			motor.setControl(mmVoltageRequest
			.withPosition(motorGoalRotations)
			.withFeedForward(velocityCompensation));
		}

        Logger.recordOutput("Hood/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Hood/velocitySetpoint", goalVelocityRadPerSec / HoodConstants.HOOD_GEAR_RATIO);
		Logger.recordOutput("Hood/SetpointDeg", Units.radiansToDegrees(goalAngle.getRadians()));

	}

	public void calibrate(){
		calibrating = true;
		setCurrentLimits(HoodConstants.CALIBRATING_CURRENT_LIMIT);
		boolean atZero = Math.abs(motor.getStatorCurrent().getValueAsDouble()) >= HoodConstants.CALIBRATION_CURRENT_THRESHOLD;
		boolean calibrated = calibrateDebouncer.calculate(atZero);
		if (calibrated){
			calibrating = false;
			motor.setPosition(Units.degreesToRotations(HoodConstants.MAX_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
		}
	}

	/**
     * sets supply and stator current limits
     * @param limitAmps the current limit for stator and supply current
     */
    public void setCurrentLimits(double limitAmps) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(limitAmps)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limitAmps);

        motor.getConfigurator().apply(limits);
    }

    @Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
		inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
	}
}
