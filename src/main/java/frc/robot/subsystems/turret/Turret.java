package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;
import frc.robot.util.ModifiedCRT;

public class Turret extends SubsystemBase implements TurretIO{
	// Super low magnitude filter for the position to make it less jittery
	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02, 0.02);

    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  	public boolean locked = false;

	private boolean calibrating;
	private Debouncer calibrationDebouncer = new Debouncer(0.5, DebounceType.kRising);

	/* ---------------- Hardware ---------------- */

	private final TalonFX motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.CANIVORE_SUB);

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

	public Turret() {
		motor.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
		config.Slot0.kP = 1.5;
		config.Slot0.kS = 0.0; // Static friction compensation
		config.Slot0.kV = 0.0; // Adjusted kV for the gear ratio
		config.Slot0.kD = 0.0; // The "Braking" term to stop overshoot
		config.Slot0.kA = 0.0;

		var mm = config.MotionMagic;
		mm.MotionMagicCruiseVelocity = Units.radiansToRotations(TurretConstants.MAX_VELOCITY) * TurretConstants.GEAR_RATIO;
		mm.MotionMagicAcceleration = Units.radiansToRotations(TurretConstants.MAX_ACCELERATION) * TurretConstants.GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
        motor.getConfigurator().apply(config);

		setNewCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT, TurretConstants.SUPPLY_CURRENT_LIMIT);

		lastGoalRad = 0.0;

		if (RobotBase.isSimulation()) {
			simState = motor.getSimState();
			turretSim = new SingleJointedArmSim(
					edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
					TurretConstants.GEAR_RATIO,
					0.01,
					0.15,
					Units.degreesToRadians(TurretConstants.MIN_ANGLE),
					Units.degreesToRadians(TurretConstants.MAX_ANGLE),
					false,
					0.0);
		}

		if (!Constants.DISABLE_SMART_DASHBOARD) {
			SmartDashboard.putData("Turret Mech", mech);
			SmartDashboard.putData("Reset Turret Position to Zero", new InstantCommand(() -> resetTurretPosition()));

			SendableChooser<InstantCommand> turretTestChooser = new SendableChooser<>();
			turretTestChooser.setDefaultOption("Turn to 0", new InstantCommand(()-> setFieldRelativeTarget(Rotation2d.fromDegrees(0), 0.0)));
			turretTestChooser.addOption("Turn to -90", new InstantCommand(()-> setFieldRelativeTarget(Rotation2d.fromDegrees(-90), 0.0)));
			turretTestChooser.addOption("Turn to 90", new InstantCommand(()-> setFieldRelativeTarget(Rotation2d.fromDegrees(90), 0.0)));
			turretTestChooser.addOption("Turn to 200", new InstantCommand(()-> setFieldRelativeTarget(Rotation2d.fromDegrees(200), 0.0)));
			turretTestChooser.addOption("Turn to -200", new InstantCommand(()-> setFieldRelativeTarget(Rotation2d.fromDegrees(-200), 0.0)));

			SmartDashboard.putData("Turret Test Positions", turretTestChooser);
		}
		SmartDashboard.putData("Set Locked", new InstantCommand(() -> {locked = !locked;}));
		//motor.setPosition(Units.degreesToRotations(238.86) * TurretConstants.GEAR_RATIO);

		motor.setPosition(0.0);
	}

	/* ---------------- Public API ---------------- */

	/**
	 * Sets the setpoint position and velocity of the turret. Call in command execute.
	 * @param angle
	 * @param velocityRadPerSec
	 */
	public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	public void resetTurretPosition() {
		inputs.positionDeg = 0.0;
	}

	/**
	 * @return If the turret is at setpoint with tolerance of 10 degrees
	 */
	public boolean atSetpoint() {
		if (locked) return true;
		return Math.abs(goalAngle.getRadians() - getPositionRad()) < Units.degreesToRadians(10.0);
	}

	/**
	 * @return Posiiton of the turret in radians
	 */
	public double getPositionRad() {
		return Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
	}

	/**
	 * @return Posiiton of the turret in degrees
	 */
	public double getPositionDeg() {
		return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
	}

	/* ---------------- Periodic ---------------- */

	@Override
	public void periodic() {
		updateInputs();
		Logger.processInputs("Turret", inputs);

		// Position extrapolation
		double lookAheadSeconds = TurretConstants.EXTRAPOLATION_TIME_CONSTANT; 
    	double futureRobotAngle = goalAngle.getRadians() + (goalVelocityRadPerSec * lookAheadSeconds);

		//Continuous wrap selection
		double best = lastGoalRad;
		boolean found = false;

		for (int i = -2; i <= 2; i++) {
			double candidate = futureRobotAngle + 2.0 * Math.PI * i;
			if (candidate < Units.degreesToRadians(TurretConstants.MIN_ANGLE) || candidate > Units.degreesToRadians(TurretConstants.MAX_ANGLE))
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
		motorGoalRotations = MathUtil.clamp(motorGoalRotations, Units.degreesToRotations(TurretConstants.MIN_ANGLE) * TurretConstants.GEAR_RATIO, Units.degreesToRotations(TurretConstants.MAX_ANGLE) * TurretConstants.GEAR_RATIO);
			
		// Multiply goal velocity by kV
		double robotTurnCompensation = goalVelocityRadPerSec * TurretConstants.FEEDFORWARD_KV * TurretConstants.GEAR_RATIO;

		// Sets motor control with feedforward
		motor.setControl(mmVoltageRequest
		.withPosition(motorGoalRotations)
		.withFeedForward(robotTurnCompensation)
		.withEnableFOC(true));

        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Turret/Voltage", motor.getMotorVoltage().getValue());
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
		turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
		turretSim.update(Constants.LOOP_TIME);

		simState.setRawRotorPosition(
				Units.radiansToRotations(turretSim.getAngleRads()) * TurretConstants.GEAR_RATIO);

		simState.setRotorVelocity(
				Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * TurretConstants.GEAR_RATIO);
	}

	@Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
		inputs.motorStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
		inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
		inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
	}

	public void setNewCurrentLimit(double stator, double supply) {
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = stator;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLimit = supply;
        limitConfig.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(limitConfig);
    }

    public double getSubsystemStatorCurrent() {
        return inputs.motorStatorCurrent;
    }

    public double getSubsystemSupplyCurrent() {
        return inputs.motorSupplyCurrent;
    }
}
